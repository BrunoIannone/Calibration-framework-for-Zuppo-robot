#include <iostream>
#include "grid.h"
#include "grid_map.h"
#include "distance_map.h"
#include "voxel.h"
#include "voxel3d.h"

#include "packet_assembler.h"
#include <vector>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include "dmap_localizer.h"
#include "draw_primitives.h"
#include "display_utils.h"
#include "structs.h"
#include "plot.h"
#include "params.h"
#include "helpers.h"

#include "hpipeline.h"
#include "spipeline.h"

#include "zuppo.h"
#include "hlidar_calibrator.h"
#include "slidar_calibrator.h"
#include <GLFW/glfw3.h>

using namespace std;

int main(int argc, char **argv)
{
  if (argc < 5)
  {
    std::cerr << RED << "Usage: ./zuppo_calibrator <path_to_image: str> <resolution: float> <path_to_data:str> <plot:bool>" << RESET << std::endl;
    return 1;
  }
  glfwInit();

  const std::string path_to_image = argv[1];
  const float resolution = std::stof(argv[2]);
  const std::string path_to_data = argv[3];
  const bool plotting = std::string(argv[4]) == "true";

  Eigen::Vector2f map_origin = parseOriginFromYaml(path_to_image);
  GridMap gmap;
  gmap.loadFromImage(path_to_image, resolution, map_origin);

  DistanceMap dmap;
  dmap.loadFromGridMap(gmap);

  Voxel voxel(VOXEL_RESOLUTION);

  DMapLocalizer loc(dmap, MAX_RANGE, USE_BILINEAR, LOCALIZE_POINT_DAMPING, KERNEL_THRESHOLD, LOCALIZE_MAX_ERROR);

  PacketAssembler assembler(path_to_data);
  const auto all_frames = assembler.getAllFrames();
  const auto all_T = assembler.getAllT();

  Zuppo zuppo(all_T.size(), assembler.getUnicycleConfig(), all_T);

  uint8_t lidar_idx = HLIDAR_IDX_FOR_ODOMETRY_CALIBRATION;

  // zuppo._lidar_vec[lidar_idx]._r_X_l.translation() = Eigen::Vector3f(-0.15f, 0.f, 0.f);
  //  zuppo._lidar_vec[lidar_idx]._r_X_l.linear() = Eigen::AngleAxisf(M_PI / 2.f, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  // zuppo._kl = -0.0001f;
  // zuppo._kr = 0.0001f;
  // zuppo._wb = 0.3f;

  auto all_framesH = all_frames[lidar_idx];
  HLidarCalibrator lidarc(zuppo, lidar_idx, CALIBRATION_MAX_ERROR, CALIBRATION_KERNEL_THRESHOLD);

  std::cerr << GREEN << "\nCurrent calibrated values: " << RESET << "\n-WB: " << lidarc._wb << "\n-KL: " << lidarc._kl << "\n-KR: " << lidarc._kr << "\n-Lidar pose:\n\n"
            << lidarc._r_X_l.matrix() << std::endl;

  Canvas dmap_canvas = grid2cv(loc._distances, true);

  for (const auto &p : dmap._obstacles) // Draw map obstacles on distance map
  {
    cv::circle(dmap_canvas, cv::Point(p.second, p.first), 2, cv::Scalar(255));
  }
  showCanvas(dmap_canvas, "Dmap", -1);

  Canvas colored_bg = cv::imread(path_to_image, cv::IMREAD_UNCHANGED);
  if (colored_bg.rows == 0 || colored_bg.channels() != 4)
  {
    throw std::runtime_error(
        "unable to load image or image does not have 4 channels");
  }

  Eigen::Isometry2f start = Eigen::Isometry2f::Identity();
  start.linear() = zuppo._lidar_vec[lidar_idx]._r_X_l.linear().topLeftCorner<2, 2>();
  bool restart = true;

  while (restart) // Horizontal calibration
  {

    std::vector<Eigen::Vector2f> odom_history;
    std::vector<Eigen::Vector2f> loc_history;

    odom_history.reserve(all_framesH.size());
    loc_history.reserve(all_framesH.size());

    replayTrajectory(all_framesH, lidarc, odom_history, zuppo, start);
    renderPlot(odom_history);
    odom_history.clear();

    const auto deskewed_frames = assembleDeskewedHFrames(assembler.getLogData(), lidar_idx, lidarc, zuppo);

    auto calibration_vector = collectHCalibrationSteps(deskewed_frames, gmap, loc, lidarc, voxel, colored_bg, plotting, odom_history, loc_history, zuppo, start);
    cout << GREEN << "\nCalibration steps collected: " << RESET << calibration_vector.size() << std::endl;

    renderPlot(odom_history, loc_history);

    std::cout << YELLOW << "\nCalibration started!" << std::endl;
    const std::vector<float> history = lidarc.calibrate(calibration_vector, CALIBRATE_MAX_ITERATIONS);

    std::cerr << GREEN << "\nCurrent calibrated values: " << RESET << "\n-WB: " << lidarc._wb << "\n-KL: " << lidarc._kl << "\n-KR: " << lidarc._kr << "\n-Lidar pose:\n"
              << lidarc._r_X_l.matrix() << std::endl;

    odom_history.clear();

    // Replay trajectory after calibration
    replayTrajectory(deskewed_frames, lidarc, odom_history, zuppo, start);

    renderPlot(odom_history, loc_history, history);

    char choice;
    std::cout << RED << "\nRestart calibration?" << RESET << " (y = Yes / s = Save checkpoint / q = Quit/Next): ";
    std::cin >> choice;
    clearInput();

    choice = std::tolower(static_cast<unsigned char>(choice));

    switch (choice)
    {
    case 's':
      saveCalibrationToLogFormat("../calibrated_log_params.txt", zuppo);

      if (plotting)
      {
        cv::destroyWindow("Localized");
        cv::destroyWindow("Odometry");
      }

      break;

    case 'q':
    {
      if (askYesNo("\nMove to slanted LiDAR calibration?"))
      {
        askAndSave(zuppo, "\nSave before changing?");
        std::cout << GREEN << "\nMoving to slanted LiDAR calibration." << RESET << std::endl;
        restart = false;

        all_framesH = deskewed_frames;

        if (plotting)
        {
          cv::destroyAllWindows();
        }
        break; // Breaks out of outer loop
      }
      else
      {
        askAndSave(zuppo, "\nSave before closing?");

        if (plotting)
        {
          cv::destroyAllWindows();
        }

        std::cout << GREEN << "\nHave fun with your halfly-calibrated Zuppo!" << RESET << std::endl;
        return 0; // Exit program
      }
    }

    case 'y':
      // Restart calibration loop
      if (plotting)
      {
        cv::destroyWindow("Localized");
        cv::destroyWindow("Odometry");
      }
      break;

    default:
      std::cout << "Unrecognized option, skipping...\n";

      if (plotting)
      {
        cv::destroyWindow("Localized");
        cv::destroyWindow("Odometry");
      }
      break;
    }
    // // --- 4. INLIERS CHOICE ---
    // std::cout << "\nWanna increase inliers? (y/n): " << std::endl;
    // std::cin >> inliers_choice;
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // if (tolower(restart_cal_choice) == 'y')
    // {
    //   MIN_INLIERS += INLIERS_INCREMENT;
    //   std::cout << "New MIN_INLIERS: " << MIN_INLIERS << std::endl;
    // }
  }

  Canvas wallfloor_canvas;
  colored_bg.copyTo(wallfloor_canvas);

  // zuppo._lidar_vec[SLIDAR_IDX_FOR_CALIBRATION]._r_X_l.linear() = Eigen::AngleAxisf(M_PI / 180.f * 60.f, Eigen::Vector3f::UnitY()).matrix() *
  //                                                                Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitZ()).matrix();
  // zuppo._lidar_vec[SLIDAR_IDX_FOR_CALIBRATION]._r_X_l.translation().z() = 0.15f;

  lidar_idx = SLIDAR_IDX_FOR_CALIBRATION;

  std::cerr << YELLOW << "\nVertical lidar matrix:\n"
            << RESET
            << zuppo._lidar_vec[SLIDAR_IDX_FOR_CALIBRATION]._r_X_l.matrix() << std::endl;

  SLidarCalibrator slidarc(zuppo, SLIDAR_IDX_FOR_CALIBRATION, gmap, loc, SCALIBRATION_MAX_ERROR, SCALIBRATION_KERNEL_THRESHOLD, SCALIBRATION_DAMPING);
  Voxel3D voxel3d(VOXEL_RESOLUTION);

  std::map<uint64_t, Eigen::Isometry2f> odom_buffer;
  std::vector<Eigen::Vector2f> loc_history;
  std::vector<Eigen::Vector2f> odom_history;

  odom_history.reserve(all_framesH.size());
  loc_history.reserve(all_framesH.size());

  collectLocalizationPoses(all_framesH, loc, voxel, colored_bg, loc_history, zuppo, odom_buffer, start);

  restart = true;

  while (restart)
  {

    const auto deskewed_sframes = assembleDeskewedSFrames(assembler.getLogData(), SLIDAR_IDX_FOR_CALIBRATION, slidarc, zuppo);

    auto calibration_vector = collectSCalibrationSteps(deskewed_sframes, voxel3d, colored_bg, zuppo, odom_buffer);

    if (plotting)
    {
      plotFloorAndWall2D(wallfloor_canvas, calibration_vector, slidarc._r_X_l, gmap);
      plotFloorAndWall3D(calibration_vector, slidarc._r_X_l, gmap);
    }

    std::cout << YELLOW << "\nCalibration started!" << std::endl;

    auto history = slidarc.calibrate(calibration_vector, SCALIBRATE_MAX_ITERATIONS);
    std::cerr << YELLOW << "\nVertical lidar matrix:\n"
              << RESET
              << zuppo._lidar_vec[SLIDAR_IDX_FOR_CALIBRATION]._r_X_l.matrix() << std::endl;
    renderPlot({}, {}, history);

    if (plotting)
    {
      plotFloorAndWall2D(wallfloor_canvas, calibration_vector, slidarc._r_X_l, gmap);
      plotFloorAndWall3D(calibration_vector, slidarc._r_X_l, gmap);
    }

    char v_choice;
    std::cout << RED << "\nRestart calibration?" << RESET
              << " (y = Yes / s = Save checkpoint / q = Quit): ";
    std::cin >> v_choice;
    clearInput();

    v_choice = std::tolower(static_cast<unsigned char>(v_choice));

    switch (v_choice)
    {
    case 's':
      saveCalibrationToLogFormat("../calibrated_log_params.txt", zuppo);

      if (plotting)
      {
        cv::destroyWindow("Floor and Walls");
      }

      break;

    case 'q':
      restart = false;

      askAndSave(zuppo, "\nSave before closing?");
      restart = false;

      if (plotting)
      {
        cv::destroyAllWindows();
      }

      glfwTerminate();

      break; // Breaks out of outer loop

    case 'y':
      if (plotting)
      {
        cv::destroyWindow("Floor and Walls");
      }

      // Proceed normally, loop restarts
      break;

    default:
      std::cout << "Unrecognized option, skipping...\n";

      if (plotting)
      {
        cv::destroyWindow("Floor and Walls");
      }

      break;
    }
  }
  std::cout << GREEN << "\nHave fun with your calibrated Zuppo!" << RESET << std::endl;
  return 0;
}