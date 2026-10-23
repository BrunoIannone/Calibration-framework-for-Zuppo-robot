#include <unistd.h>
#include <cmath>
#include <iostream>

#include <GLFW/glfw3.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "draw_primitives.h"

#include "params.h"
#include "structs.h"
#include "grid_map.h"

#include <fstream>
#include <limits>
#include <algorithm>

// ------------------------------------------------------------------
// Minimal, dependency-free SVG line-plot exporter.
// Draws axes, ticks and one or more polylines directly from data,
// with no dependency on ImPlot's rendering pipeline.
// ------------------------------------------------------------------
namespace SvgPlot
{
    struct Series
    {
        std::vector<double> xs;
        std::vector<double> ys;
        std::string color; // e.g. "#e74c3c"
        std::string label;
    };

    // --- "nice number" tick helpers -----------------------------------

    inline double niceStep(double rawStep)
    {
        if (rawStep <= 0.0)
            return 1.0;
        const double exponent = std::floor(std::log10(rawStep));
        const double fraction = rawStep / std::pow(10.0, exponent);

        double niceFraction;
        if (fraction <= 1.0)
            niceFraction = 1.0;
        else if (fraction <= 2.0)
            niceFraction = 2.0;
        else if (fraction <= 5.0)
            niceFraction = 5.0;
        else
            niceFraction = 10.0;

        return niceFraction * std::pow(10.0, exponent);
    }

    // Generates "nice" tick values covering [min, max], roughly targetCount of them.
    inline std::vector<double> computeNiceTicks(double min_v, double max_v, int targetCount = 10)
    {
        if (max_v <= min_v)
        {
            max_v = min_v + 1.0;
        }

        const double range = max_v - min_v;
        const double rawStep = range / std::max(1, targetCount - 1);
        const double step = niceStep(rawStep);

        const double niceMin = std::floor(min_v / step) * step;
        const double niceMax = std::ceil(max_v / step) * step;

        std::vector<double> ticks;
        for (double v = niceMin; v <= niceMax + step * 0.5; v += step)
            ticks.push_back(v);

        return ticks;
    }

    // Decimal places to show, based on the tick step size.
    inline int decimalsForStep(double step)
    {
        if (step <= 0.0)
            return 2;
        int decimals = -static_cast<int>(std::floor(std::log10(step)));
        return std::max(0, decimals);
    }

    inline std::string formatTick(double value, int decimals)
    {
        // Avoid "-0.0" from floating point noise near zero
        if (std::abs(value) < 1e-9)
            value = 0.0;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(decimals) << value;
        return oss.str();
    }

    // --------------------------------------------------------------------

    inline void save(const std::string &filename,
                     const std::vector<Series> &series,
                     const std::string &x_label,
                     const std::string &y_label,
                     int width = 800,
                     int height = 600)
    {
        // Increased left margin so long Y-axis numbers don't get cut off in SVG/PDF
        constexpr int margin_left = 110;
        constexpr int margin_right = 30;
        constexpr int margin_top = 30;
        constexpr int margin_bottom = 60;

        // How dense the ticks should be. Bump these up/down to taste.
        constexpr int target_x_ticks = 10;
        constexpr int target_y_ticks = 15;

        // Font sizes.
        constexpr int font_tick = 25;
        constexpr int font_axis_label = 20;
        constexpr int font_legend = 20;

        const std::string grid_color = "#e0e0e0"; // light gray

        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();

        bool has_data = false;
        for (const auto &s : series)
        {
            for (double v : s.xs)
            {
                min_x = std::min(min_x, v);
                max_x = std::max(max_x, v);
                has_data = true;
            }
            for (double v : s.ys)
            {
                min_y = std::min(min_y, v);
                max_y = std::max(max_y, v);
                has_data = true;
            }
        }
        if (!has_data)
            return;

        // Check if x-axis is integer discrete steps (e.g. Iterations)
        bool is_x_integer = (x_label == "Iterations");

        if (is_x_integer)
        {
            // Lock integer domain for iterations without fractional padding
            min_x = std::floor(min_x);
            max_x = std::ceil(max_x);
            if (max_x <= min_x)
                max_x = min_x + 1.0;
        }
        else
        {
            if (max_x - min_x < 1e-9)
            {
                max_x += 1.0;
                min_x -= 1.0;
            }
            const double pad_x = (max_x - min_x) * 0.05;
            min_x -= pad_x;
            max_x += pad_x;
        }

        if (max_y - min_y < 1e-9)
        {
            max_y += 1.0;
            min_y -= 1.0;
        }
        const double pad_y = (max_y - min_y) * 0.05;
        min_y -= pad_y;
        max_y += pad_y;

        const double plot_w = width - margin_left - margin_right;
        const double plot_h = height - margin_top - margin_bottom;

        auto toPx = [&](double x, double y)
        {
            const double px = margin_left + (x - min_x) / (max_x - min_x) * plot_w;
            const double py = margin_top + (1.0 - (y - min_y) / (max_y - min_y)) * plot_h;
            return std::make_pair(px, py);
        };

        std::ofstream f(filename);
        if (!f.is_open())
        {
            std::cerr << "SvgPlot::save() failed to open " << filename << std::endl;
            return;
        }

        f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
          << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " " << height
          << "\" font-family=\"sans-serif\">\n";

        // Background
        f << "  <rect x=\"0\" y=\"0\" width=\"" << width << "\" height=\"" << height
          << "\" fill=\"white\"/>\n";

        // Plot border
        f << "  <rect x=\"" << margin_left << "\" y=\"" << margin_top
          << "\" width=\"" << plot_w << "\" height=\"" << plot_h
          << "\" fill=\"none\" stroke=\"black\" stroke-width=\"1\"/>\n";

        // --- Axis ticks + gridlines (nice numbers instead of raw linear interpolation) ---

        // X ticks + vertical gridlines
        {
            std::vector<double> xticks;
            if (is_x_integer)
            {
                double step = niceStep((max_x - min_x) / std::max(1, target_x_ticks - 1));
                step = std::max(1.0, std::round(step)); // keep integer-valued steps
                double startv = std::floor(min_x / step) * step;
                for (double v = startv; v <= max_x + step * 0.5; v += step)
                    xticks.push_back(v);
            }
            else
            {
                xticks = computeNiceTicks(min_x, max_x, target_x_ticks);
            }

            const double xstep = xticks.size() > 1 ? (xticks[1] - xticks[0]) : 1.0;
            const int xdecimals = is_x_integer ? 0 : decimalsForStep(xstep);

            for (double xv : xticks)
            {
                if (xv < min_x - 1e-9 || xv > max_x + 1e-9)
                    continue; // keep ticks within the plotted range

                auto [px, py_bottom] = toPx(xv, min_y);

                // Vertical gridline (drawn first so it sits behind data series drawn later)
                f << "  <line x1=\"" << px << "\" y1=\"" << margin_top
                  << "\" x2=\"" << px << "\" y2=\"" << margin_top + plot_h
                  << "\" stroke=\"" << grid_color << "\" stroke-width=\"1\"/>\n";

                // Tick mark
                f << "  <line x1=\"" << px << "\" y1=\"" << margin_top + plot_h
                  << "\" x2=\"" << px << "\" y2=\"" << margin_top + plot_h + 5
                  << "\" stroke=\"black\"/>\n";

                f << "  <text x=\"" << px << "\" y=\"" << margin_top + plot_h + 20
                  << "\" font-size=\"" << font_tick << "\" text-anchor=\"middle\">";

                if (is_x_integer)
                    f << static_cast<long long>(std::round(xv));
                else
                    f << formatTick(xv, xdecimals);

                f << "</text>\n";
            }
        }

        // Y ticks + horizontal gridlines
        {
            std::vector<double> yticks = computeNiceTicks(min_y, max_y, target_y_ticks);
            const double ystep = yticks.size() > 1 ? (yticks[1] - yticks[0]) : 1.0;
            const int ydecimals = decimalsForStep(ystep);

            for (double yv : yticks)
            {
                if (yv < min_y - 1e-9 || yv > max_y + 1e-9)
                    continue;

                auto [px_left, py] = toPx(min_x, yv);

                // Horizontal gridline
                f << "  <line x1=\"" << margin_left << "\" y1=\"" << py
                  << "\" x2=\"" << margin_left + plot_w << "\" y2=\"" << py
                  << "\" stroke=\"" << grid_color << "\" stroke-width=\"1\"/>\n";

                // Tick mark
                f << "  <line x1=\"" << margin_left - 5 << "\" y1=\"" << py
                  << "\" x2=\"" << margin_left << "\" y2=\"" << py
                  << "\" stroke=\"black\"/>\n";

                f << "  <text x=\"" << margin_left - 10 << "\" y=\"" << py + 4
                  << "\" font-size=\"" << font_tick << "\" text-anchor=\"end\">"
                  << formatTick(yv, ydecimals) << "</text>\n";
            }
        }

        // Axis labels (Offset adjusted for the larger margin)
        f << "  <text x=\"" << margin_left + plot_w / 2 << "\" y=\"" << height - 10
          << "\" font-size=\"" << font_axis_label << "\" text-anchor=\"middle\">" << x_label << "</text>\n";

        f << "  <text x=\"20\" y=\"" << margin_top + plot_h / 2
          << "\" font-size=\"" << font_axis_label << "\" text-anchor=\"middle\" transform=\"rotate(-90 20,"
          << margin_top + plot_h / 2 << ")\">" << y_label << "</text>\n";

        // Data series (drawn after grid so lines sit on top of it)
        for (const auto &s : series)
        {
            if (s.xs.empty())
                continue;

            f << "  <polyline fill=\"none\" stroke=\"" << s.color
              << "\" stroke-width=\"3\" points=\"";
            for (size_t i = 0; i < s.xs.size(); ++i)
            {
                auto [px, py] = toPx(s.xs[i], s.ys[i]);
                f << px << "," << py << " ";
            }
            f << "\"/>\n";
        }

        // Legend
        double legend_y = margin_top + 15;
        for (const auto &s : series)
        {
            if (s.label.empty())
                continue;
            f << "  <line x1=\"" << margin_left + plot_w - 90 << "\" y1=\"" << legend_y
              << "\" x2=\"" << margin_left + plot_w - 65 << "\" y2=\"" << legend_y
              << "\" stroke=\"" << s.color << "\" stroke-width=\"2\"/>\n";
            f << "  <text x=\"" << margin_left + plot_w - 60 << "\" y=\"" << legend_y + 4
              << "\" font-size=\"" << font_legend << "\">" << s.label << "</text>\n";
            legend_y += 16;
        }

        f << "</svg>\n";
        f.close();

        std::cout << "Saved plot: " << filename << std::endl;
    }
}

// Writes chi_history.svg from the Chi convergence history.
void saveChiSvg(const std::vector<float> &history, const std::string &filename = "chi_history.svg")
{
    if (history.empty())
        return;

    SvgPlot::Series s;
    s.color = "#2980b9";
    s.label = "F(x)";
    s.xs.reserve(history.size());
    s.ys.reserve(history.size());
    for (size_t i = 0; i < history.size(); ++i)
    {
        s.xs.push_back(static_cast<double>(i));
        s.ys.push_back(static_cast<double>(history[i]));
    }

    SvgPlot::save(filename, {s}, "Iterations", "Cost");
}

// Writes trajectories.svg from odometry and localized trajectories.
void saveTrajectoriesSvg(const std::vector<Eigen::Vector2f> &odom_history,
                         const std::vector<Eigen::Vector2f> &cal_odom_history,
                         const std::string &filename = "trajectories.svg")
{
    if (odom_history.empty() && cal_odom_history.empty())
        return;

    std::vector<SvgPlot::Series> series;

    if (!odom_history.empty())
    {
        SvgPlot::Series s;
        s.color = "#e74c3c";
        s.label = "Odometry";
        s.xs.reserve(odom_history.size());
        s.ys.reserve(odom_history.size());
        for (const auto &p : odom_history)
        {
            s.xs.push_back(static_cast<double>(p.x()));
            s.ys.push_back(static_cast<double>(p.y()));
        }
        series.push_back(std::move(s));
    }

    if (!cal_odom_history.empty())
    {
        SvgPlot::Series s;
        s.color = "#27ae60";
        s.label = "Localized";
        s.xs.reserve(cal_odom_history.size());
        s.ys.reserve(cal_odom_history.size());
        for (const auto &p : cal_odom_history)
        {
            s.xs.push_back(static_cast<double>(p.x()));
            s.ys.push_back(static_cast<double>(p.y()));
        }
        series.push_back(std::move(s));
    }

    SvgPlot::save(filename, series, "X [m]", "Y [m]");
}

void plotChi(const std::vector<float> &history)
{
    if (history.empty())
        return;

    ImGui::Begin("Chi History");

    if (ImPlot::BeginPlot("Chi History", ImVec2(-1, -1)))
    {
        ImPlot::SetupAxes("Iterations", "Chi");

        ImPlot::PlotLine(
            "Chi",
            history.data(),     // Pointer to the first float
            (int)history.size() // Number of points
        );

        ImPlot::EndPlot();
    }

    ImGui::End();
}

void plotTrajectory(const std::vector<Eigen::Vector2f> &odom_history, const char *label_id)
{
    if (odom_history.empty())
        return;
    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 3.0f); // 3.0f is the line weight in pixels
    ImPlot::PlotLineG(
        label_id,
        [](int idx, void *data)
        {
            auto *h = static_cast<const std::vector<Eigen::Vector2f> *>(data);
            return ImPlotPoint((*h)[idx].x(), (*h)[idx].y());
        },
        (void *)&odom_history,
        static_cast<int>(odom_history.size()));
}

void renderPlot(const std::vector<Eigen::Vector2f> &odom_history, const std::vector<Eigen::Vector2f> &cal_odom_history, const std::vector<float> &history)
{
    // Export SVGs directly from the data (independent of ImPlot's GUI
    // rendering pipeline), so they're written as soon as the data is ready.
    saveChiSvg(history);
    saveTrajectoriesSvg(odom_history, cal_odom_history);

    if (!glfwInit())
    {
        std::cerr << "plot(): failed to initialize GLFW" << std::endl;
        return;
    }

    GLFWwindow *prev_context = glfwGetCurrentContext();
    ImGuiContext *prev_imgui_ctx = ImGui::GetCurrentContext();
    ImPlotContext *prev_implot_ctx = ImPlot::GetCurrentContext();

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_FALSE);

    GLFWwindow *window = glfwCreateWindow(1280, 720, "Plot", NULL, NULL);
    if (!window)
    {
        std::cerr << "plot(): failed to create GLFW window" << std::endl;
        if (prev_context)
            glfwMakeContextCurrent(prev_context);
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    ImGuiContext *imgui_ctx = ImGui::CreateContext();
    ImGui::SetCurrentContext(imgui_ctx);
    ImPlotContext *implot_ctx = ImPlot::CreateContext();
    ImPlot::SetCurrentContext(implot_ctx);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    ImGui::StyleColorsDark();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // #### PLOT ####
        if (!history.empty())
            plotChi(history);

        if (!odom_history.empty() || !cal_odom_history.empty())
        {
            ImGui::Begin("Trajectories plot");
            if (ImPlot::BeginPlot("Trajectories", ImVec2(-1, -1)))
            {
                ImPlot::SetupAxes("X [m]", "Y [m]");

                plotTrajectory(odom_history, "Odometry");
                plotTrajectory(cal_odom_history, "Localized");

                ImPlot::EndPlot();
            }
            ImGui::End();
        }
        // ##############

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext(implot_ctx);
    ImGui::DestroyContext(imgui_ctx);

    glfwDestroyWindow(window);

    // Handle previous context safely
    if (prev_context)
        glfwMakeContextCurrent(prev_context);
    if (prev_imgui_ctx)
        ImGui::SetCurrentContext(prev_imgui_ctx);
    if (prev_implot_ctx)
        ImPlot::SetCurrentContext(prev_implot_ctx);
}

void plotFloorAndWall2D(Canvas colored_bg,
                        const std::vector<SCalibrationStep> &calibration_vector,
                        const Eigen::Isometry3f &r_X_l,
                        const GridMap &gmap)
{
    std::vector<Eigen::Vector2f> wall_points;
    std::vector<Eigen::Vector2f> floor_points;

    for (const auto &step : calibration_vector)
    {
        const Eigen::Matrix3Xf world_points = step.w_T_r * r_X_l * step.l_z;

        for (int i = 0; i < world_points.cols(); ++i)
        {
            const float z = world_points(2, i);
            const bool is_wall = (z >= Z_WALL_MIN);
            const bool is_floor = (z >= Z_FLOOR_MIN && z < Z_FLOOR_MAX);

            if (!is_wall && !is_floor)
                continue;

            const Eigen::Vector2f p(world_points(0, i), world_points(1, i));

            if (!gmap.inside(p))
                continue;

            const Eigen::Vector2i pg = gmap.worldToGrid(p);
            if (gmap.at(pg.y(), pg.x()) == gmap.UNKNOWN)
                continue;

            if (is_wall)
            {
                wall_points.push_back(p);
            }
            else
            {
                floor_points.push_back(p);
            }
        }
    }

    if (!floor_points.empty())
    {
        drawPoints(colored_bg, gmap, floor_points, cv::Scalar(255, 0, 0));
    }
    if (!wall_points.empty())
    {
        drawPoints(colored_bg, gmap, wall_points, cv::Scalar(0, 0, 255));
    }

    cv::imshow("Floor and Walls", colored_bg);
    cv::waitKey(20000);
}

void plotFloorAndWall3D(
    const std::vector<SCalibrationStep> &calibration_vector,
    const Eigen::Isometry3f &r_X_l,
    const GridMap &gmap)
{
    std::vector<glm::vec3> wall_points;
    std::vector<glm::vec3> floor_points;
    double sum = 0.0, sum_sq = 0.0;
    size_t floor_count = 0;
    for (const auto &step : calibration_vector)
    {
        const Eigen::Matrix3Xf world_points = step.w_T_r * r_X_l * step.l_z;

        for (int i = 0; i < world_points.cols(); ++i)
        {
            const float z = world_points(2, i);
            const bool is_wall = (z >= Z_FLOOR_MAX);
            const bool is_floor = (z >= Z_FLOOR_MIN && z < Z_FLOOR_MAX);

            if (!is_wall && !is_floor)
                continue;

            const Eigen::Vector2f p(world_points(0, i), world_points(1, i));

            if (!gmap.inside(p))
                continue;

            const Eigen::Vector2i pg = gmap.worldToGrid(p);
            if (gmap.at(pg.y(), pg.x()) == gmap.UNKNOWN)
                continue;

            if (is_wall)
            {
                wall_points.emplace_back(p.y(), z, p.x());
            }
            else
            {
                floor_points.emplace_back(p.y(), z, p.x());
                sum += z;
                sum_sq += static_cast<double>(z) * z;
                ++floor_count;
            }
        }
    }

    static bool initialized = false;
    if (!initialized)
    {
        polyscope::init();
        initialized = true;
    }

    auto *wall_cloud = polyscope::registerPointCloud("Wall", wall_points);
    wall_cloud->setPointColor({1.0f, 0.0f, 0.0f}); // Red
    wall_cloud->setPointRadius(0.001f);

    auto *floor_cloud = polyscope::registerPointCloud("Floor", floor_points);
    floor_cloud->setPointColor({0.0f, 0.0f, 1.0f}); // Blue
    floor_cloud->setPointRadius(0.001f);

    const double mean = sum / floor_count;
    const double rms = std::sqrt(sum_sq / floor_count);
    const double var = sum_sq / floor_count - mean * mean;
    const double std = std::sqrt(std::max(0.0, var));

    std::cout << "N: " << floor_count
              << ", Mean: " << mean
              << ", RMS: " << rms
              << ", STD: " << std << std::endl;

    // polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::Tile;  // or TileReflection, ShadowOnly
    // polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
    // polyscope::options::groundPlaneHeight = 0.0f;  // y = 0, i.e. world z = 0

    polyscope::show();
}

// void plotFloorAndWall3D(                  // ->DEPRECATED
//     const std::vector<SCalibrationStep> &calibration_vector,
//     const Eigen::Isometry3f &r_X_l,
//     const GridMap &gmap)
// {
//     // Open persistent gnuplot process.
//     static FILE *gp = nullptr;

//     if (!gp)
//     {
//         gp = popen("gnuplot -persist", "w");

//         if (!gp)
//         {
//             std::cerr << "Failed to start gnuplot\n";
//             return;
//         }

//         fprintf(gp, "set title 'Floor / Wall Points'\n");
//         fprintf(gp, "set xlabel 'X'\n");
//         fprintf(gp, "set ylabel 'Y'\n");
//         fprintf(gp, "set zlabel 'Z'\n");

//         fprintf(gp, "set grid\n");
//         fprintf(gp, "set key outside\n");

//         fprintf(gp, "set view 60, 45\n");

//         fprintf(gp, "set view equal xyz\n");

//         fflush(gp);
//     }

//     std::vector<Eigen::Vector3f> wall_points;
//     std::vector<Eigen::Vector3f> floor_points;

//     for (const auto &step : calibration_vector)
//     {
//         const Eigen::Matrix3Xf world_points = step.w_T_r * r_X_l * step.l_z;

//         for (int i = 0; i < world_points.cols(); ++i)
//         {
//             const float z = world_points(2, i);
//             const bool is_wall = (z > Z_WALL_MIN && z < Z_WALL_MAX);
//             const bool is_floor = (z > Z_FLOOR_MIN && z < Z_FLOOR_MAX);

//             if (!is_wall && !is_floor)
//                 continue;

//             const Eigen::Vector2f p(world_points(0, i), world_points(1, i));

//             if (!gmap.inside(p))
//                 continue;

//             const Eigen::Vector2i pg = gmap.worldToGrid(p);
//             if (gmap.at(pg.y(), pg.x()) == gmap.UNKNOWN)
//                 continue;

//             if (is_wall)
//                 wall_points.emplace_back(p.x(), p.y(), z);

//             else
//             {
//                 floor_points.emplace_back(p.x(), p.y(), z);
//             }
//         }
//     }

//     // -------------------------------------------------------------
//     // Tell gnuplot what we're plotting.
//     //
//     // '-' means that the following data comes from stdin.
//     // An empty line terminates one data set.
//     // -------------------------------------------------------------

//     fprintf(gp,
//             "splot "
//             "'-' using 1:2:3 with points pt 7 ps 0.4 lc rgb 'red' "
//             "title 'Wall', "
//             "'-' using 1:2:3 with points pt 7 ps 0.4 lc rgb 'blue' "
//             "title 'Floor'\n");

//     // Wall data
//     for (const auto &p : wall_points)
//     {
//         fprintf(gp, "%f %f %f\n",
//                 p.x(), p.y(), p.z());
//     }

//     fprintf(gp, "e\n");

//     // Floor data
//     for (const auto &p : floor_points)
//     {
//         fprintf(gp, "%f %f %f\n",
//                 p.x(), p.y(), p.z());
//     }

//     fprintf(gp, "e\n");

//     fflush(gp);
// }