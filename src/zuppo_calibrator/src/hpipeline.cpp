#include "hpipeline.h"

#include "packet_assembler.h"
#include "params.h"
#include "display_utils.h"
#include "helpers.h"
#include "packet_assembler.h"
#include "draw_primitives.h"

std::vector<HCalibrationStep> collectHCalibrationSteps(
    const std::vector<Frame> &all_framesH,
    const GridMap &gmap,
    DMapLocalizer &loc,
    HLidarCalibrator &hlidarc,
    Voxel &voxel,
    const Canvas &background_image,
    const bool plotting,
    std::vector<Eigen::Vector2f> &odom_history,
    std::vector<Eigen::Vector2f> &loc_history,
    const Zuppo &zuppo,
    const Eigen::Isometry2f &w_X_l0)
{
    std::vector<HCalibrationStep> calibration_vector;
    Chunk chunk;

    Canvas odom_canvas, loc_canvas;
    background_image.copyTo(odom_canvas);
    background_image.copyTo(loc_canvas);

    Eigen::Isometry2f w_X_l = w_X_l0;
    Eigen::Isometry2f w_X_l_prev = w_X_l;
    Eigen::Isometry2f w_X_l_odom = w_X_l;
    Eigen::Isometry2f w_X_l_calib = w_X_l;

    loc.X = w_X_l;

    loc_history.push_back(w_X_l.translation());
    odom_history.push_back(w_X_l_odom.translation());

    auto prev_frame = all_framesH[FRAME_START_IDX];
    double start_time = 0;
    bool is_timer_running = false;
    bool localized_curr = false;
    bool localized_prev = true;
    const Eigen::Isometry2f r_X_l_inv = hlidarc._r_X_l.inverse();

    for (size_t i = 1 + FRAME_START_IDX; i < all_framesH.size(); ++i)
    {
        printProgressStats(i, all_framesH.size(), start_time, is_timer_running);

        const auto &curr_frame = all_framesH[i];
        const int dt = curr_frame.servo.timestamp - prev_frame.servo.timestamp;

        const Eigen::Vector2f ticks(
            static_cast<int32_t>(curr_frame.servo.ticks[LEFT_WHEEL_IDX] - prev_frame.servo.ticks[LEFT_WHEEL_IDX]),
            static_cast<int32_t>(curr_frame.servo.ticks[RIGHT_WHEEL_IDX] - prev_frame.servo.ticks[RIGHT_WHEEL_IDX]));

        const Eigen::Isometry2f dl_isometry = r_X_l_inv * zuppo.hOdom(Eigen::Vector3f(hlidarc._kl, hlidarc._kr, hlidarc._wb), ticks) * hlidarc._r_X_l;

        w_X_l = w_X_l * dl_isometry;
        w_X_l_odom = w_X_l_odom * dl_isometry;
        odom_history.push_back(w_X_l_odom.translation());

        for (const auto &z : curr_frame.scan.points)
        {
            voxel.addPoint(z);
        }

        std::vector<Eigen::Vector2f> zs_voxel = voxel.voxelize();

        if (plotting)
        {
            renderDebugUI("Odometry", odom_canvas, gmap, w_X_l_odom, {});
        }

        loc.X = w_X_l;
        loc.localize(zs_voxel, LOCALIZE_MAX_ITERATIONS);

        const Eigen::Vector2f loc_pose = loc.X.translation();
        loc_history.push_back(loc_pose);
        w_X_l = loc.X;
        if (loc._num_inliers > MIN_INLIERS)
        {
            w_X_l_calib = w_X_l;
            localized_curr = true;
        }
        else
        {
            std::cout << YELLOW << "WARNING: Below inliers threshold" << RESET << std::endl;
            localized_curr = false;
            chunk.reset();
        }

        if (localized_curr && localized_prev)
        {
            const Eigen::Vector2f variance = chunk.addVelocity(hlidarc.getState(), dt, ticks);

            if (std::abs(chunk._currVel.x()) == 0 && std::abs(chunk._currVel.y()) == 0)
            {
                std::cout << YELLOW << "NO MOTION DETECTED" << RESET << std::endl;
                chunk.reset();
            }
            else if (variance.x() > VARIANCE_THRESHOLD || variance.y() > VARIANCE_THRESHOLD)
            {
                std::cout << RED << "VARIANCE ABOVE THRESHOLD" << RESET << std::endl;
                chunk.reset();
            }
            else if (dt > MAX_DT)
            {
                std::cout << YELLOW << "DT OVER THRESHOLD (" << dt << " us)" << RESET << std::endl;
                chunk.reset();
            }
            else
            {
                if (chunk.addCs(w_X_l_prev, w_X_l_calib, ticks, t2v(dl_isometry)))
                {
                    std::cout << GREEN << "SAVING" << RESET << std::endl;

                    calibration_vector.push_back(std::move(chunk._step));

                    if (plotting)
                    {
                        const Eigen::Vector2i px_a = gmap.worldToGrid(chunk._step.T_a.translation());
                        const Eigen::Vector2i px_b = gmap.worldToGrid(chunk._step.T_b.translation());
                        drawLine(loc_canvas, px_a, px_b, cv::Scalar(255, 0, 0), 1);
                    }
                    chunk.reset();
                }
            }
        }

        if (plotting)
        {
            renderDebugUI("Localized", loc_canvas, gmap, loc.X, zs_voxel);
            cv::waitKey(1); // Non-blocking for smooth playback
        }

        if (localized_curr)
        {
            w_X_l_prev = w_X_l_calib;
            localized_prev = true;
        }
        else
        {
            localized_prev = false;
        }

        prev_frame = curr_frame;
        voxel._voxel_map.clear();
    }
    return calibration_vector;
}

void replayTrajectory(
    const std::vector<Frame> &all_framesH,
    const HLidarCalibrator &hlidarc,
    std::vector<Eigen::Vector2f> &odom_history,
    const Zuppo &zuppo,
    const Eigen::Isometry2f &w_X_l0)
{
    Eigen::Isometry2f w_X_l = w_X_l0;
    odom_history.push_back(w_X_l.translation());

    const Eigen::Isometry2f r_X_l_inv = hlidarc._r_X_l.inverse();

    auto prev_frame = all_framesH[FRAME_START_IDX];

    for (size_t i = 1 + FRAME_START_IDX; i < all_framesH.size(); ++i)
    {
        const auto curr_frame = all_framesH[i];

        const Eigen::Vector2f ticks(
            static_cast<int32_t>(curr_frame.servo.ticks[LEFT_WHEEL_IDX] - prev_frame.servo.ticks[LEFT_WHEEL_IDX]),
            static_cast<int32_t>(curr_frame.servo.ticks[RIGHT_WHEEL_IDX] - prev_frame.servo.ticks[RIGHT_WHEEL_IDX]));

        const Eigen::Isometry2f dl_isometry = r_X_l_inv * zuppo.hOdom(Eigen::Vector3f(hlidarc._kl, hlidarc._kr, hlidarc._wb), ticks) * hlidarc._r_X_l;

        w_X_l = w_X_l * dl_isometry;
        odom_history.push_back(w_X_l.translation());

        prev_frame = curr_frame;
    }
}

std::vector<Scan> assembleDeskewedScans(
    const LogData &data,
    const std::vector<LidarPacket> &pkts,
    const int lidar_idx,
    const HLidarCalibrator &hlidarc,
    const Zuppo &zuppo)
{
    std::vector<Scan> scans;
    Scan current;
    PacketAssembler assembler;

    int prev_idx = -1;
    uint64_t t_0 = 0;
    ServoEntry s_start;

    const Eigen::Isometry2f r_X_l = hlidarc._r_X_l;
    const Eigen::Isometry2f l_X_r = r_X_l.inverse();

    for (size_t i = 0; i < pkts.size(); ++i)
    {
        if (i % 5000 == 0 || i == pkts.size() - 1)
        {
            std::cout << "Processing packet " << i + 1 << " / " << pkts.size() << "\r" << std::flush;
        }
        const auto &pkt = pkts[i];
        if (pkt.idx != lidar_idx)
            continue;

        bool new_scan = false;
        if (prev_idx >= 0 && pkts[prev_idx].start_angle > pkts[prev_idx].end_angle)
        {
            new_scan = true;
        }
        prev_idx = (int)i;

        if (new_scan)
        {
            if (!current.points.empty())
            {
                scans.push_back(std::move(current));
            }
            current.points.clear();
        }

        if (current.points.empty())
        {
            current.timestamp = pkt.timestamp;
            t_0 = pkt.timestamp; // Projection center at the initial pose
            s_start = assembler.nearestServo(data.servo, t_0);
        }

        int d_angle = (int)pkt.end_angle - (int)pkt.start_angle;
        if (d_angle < 0)
            d_angle += 36000;
        const float step = d_angle / 11.f;

        // Estimated time length
        const double dt_packet_us = (static_cast<double>(d_angle) / pkt.speed) * 1e6;

        const uint64_t t_packet_start = pkt.timestamp;
        const uint64_t t_packet_end = pkt.timestamp + static_cast<uint64_t>(dt_packet_us);

        const ServoEntry s_packet_start = assembler.nearestServo(data.servo, t_packet_start);
        const ServoEntry s_packet_end = assembler.nearestServo(data.servo, t_packet_end);

        // Compute accumulated ticks from t_0 up to the start and end of this specific packet
        const Eigen::Vector2i ticks_at_packet_start(
            static_cast<int32_t>(s_packet_start.ticks[LEFT_WHEEL_IDX] - s_start.ticks[LEFT_WHEEL_IDX]),
            static_cast<int32_t>(s_packet_start.ticks[RIGHT_WHEEL_IDX] - s_start.ticks[RIGHT_WHEEL_IDX]));

        const Eigen::Vector2i ticks_at_packet_end(
            static_cast<int32_t>(s_packet_end.ticks[LEFT_WHEEL_IDX] - s_start.ticks[LEFT_WHEEL_IDX]),
            static_cast<int32_t>(s_packet_end.ticks[RIGHT_WHEEL_IDX] - s_start.ticks[RIGHT_WHEEL_IDX]));

        // Process the PACKET_SIZE points within the packet
        for (int j = 0; j < PACKET_SIZE; ++j)
        {
            const uint16_t dist_mm = pkt.points[j].distance;
            if (dist_mm < 200 || dist_mm > 12000 || pkt.points[j].confidence < 10)
                continue;

            const float angle_hdeg = pkt.start_angle + j * step;
            const float angle_rad = angle_hdeg * (float)M_PI / 18000.f;
            const float d = dist_mm * 1e-3f;

            const Eigen::Vector2f p_raw(d * std::cos(angle_rad), -d * std::sin(angle_rad));

            const float fraction = j / 11.0f;
            const Eigen::Vector2f interpolated_ticks =
                ticks_at_packet_start.cast<float>() +
                fraction * (ticks_at_packet_end - ticks_at_packet_start).cast<float>();

            const Eigen::Isometry2f dl_isometry = l_X_r * zuppo.hOdom(Eigen::Vector3f(hlidarc._kl, hlidarc._kr, hlidarc._wb), interpolated_ticks) * r_X_l;
            const Eigen::Vector2f p_deskewed = dl_isometry * p_raw;

            current.points.push_back(p_deskewed);
        }
    }

    if (!current.points.empty())
    {
        scans.push_back(std::move(current));
    }

    return scans;
}

std::vector<Frame> assembleDeskewedHFrames(const LogData &data, uint8_t lidar_idx, const HLidarCalibrator &hlidarc, const Zuppo &zuppo)
{
    PacketAssembler assembler;

    auto scans = assembleDeskewedScans(data, data.lidar, lidar_idx, hlidarc, zuppo);
    std::vector<Frame> frames;
    frames.reserve(scans.size());
    for (auto &scan : scans)
    {
        Frame f;
        f.scan = std::move(scan);
        // f.odom = assembler.nearestOdom(data.odom, f.scan.timestamp);
        f.servo = assembler.nearestServo(data.servo, f.scan.timestamp);
        frames.push_back(std::move(f));
    }
    return frames;
}
