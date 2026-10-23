#include "spipeline.h"
#include "packet_assembler.h"

void collectLocalizationPoses(
    const std::vector<Frame> &all_framesH,
    DMapLocalizer &loc,
    Voxel &voxel,
    const Canvas &background_image,
    std::vector<Eigen::Vector2f> &loc_history,
    const Zuppo &zuppo,
    std::map<uint64_t, Eigen::Isometry2f> &odom_buffer,
    const Eigen::Isometry2f &w_X_l0)
{

    Canvas loc_canvas;
    background_image.copyTo(loc_canvas);

    Eigen::Isometry2f w_X_lh = w_X_l0;

    loc.X = w_X_lh;

    odom_buffer[all_framesH[FRAME_START_IDX].scan.timestamp] = w_X_lh;

    const Eigen::Isometry2f r_X_lh(w_X_lh);
    const Eigen::Isometry2f r_X_lh_inv = r_X_lh.inverse();

    loc_history.push_back(w_X_lh.translation());

    auto prev_frame = all_framesH[FRAME_START_IDX];

    double start_time = 0;
    bool is_timer_running = false;

    for (size_t i = 1 + FRAME_START_IDX; i < all_framesH.size(); ++i)
    {
        printProgressStats(i, all_framesH.size(), start_time, is_timer_running);

        const auto &curr_frame = all_framesH[i];

        const Eigen::Vector2f ticks(
            static_cast<int32_t>(curr_frame.servo.ticks[LEFT_WHEEL_IDX] - prev_frame.servo.ticks[LEFT_WHEEL_IDX]),
            static_cast<int32_t>(curr_frame.servo.ticks[RIGHT_WHEEL_IDX] - prev_frame.servo.ticks[RIGHT_WHEEL_IDX]));

        // const Eigen::Isometry2f dx_isometry = v2t<float>(zuppo.hOdom(Eigen::Vector3f(zuppo._kl, zuppo._kr, zuppo._wb), ticks));
        const Eigen::Isometry2f dl_isometry = r_X_lh_inv * zuppo.hOdom(Eigen::Vector3f(zuppo._kl, zuppo._kr, zuppo._wb), ticks) * r_X_lh;

        w_X_lh = w_X_lh * dl_isometry;

        for (const auto &z : curr_frame.scan.points)
        {
            voxel.addPoint(z);
        }

        std::vector<Eigen::Vector2f> zs_voxel;
        for (const auto &[_, value] : voxel._voxel_map)
        {
            Eigen::Vector2f rp_mean = value.mean();
            zs_voxel.push_back(rp_mean);
        }

        loc.X = w_X_lh;
        loc.localize(zs_voxel, LOCALIZE_MAX_ITERATIONS);

        const Eigen::Vector2f loc_pose = loc.X.translation();
        loc_history.push_back(loc_pose);

        w_X_lh = loc.X;
        odom_buffer[curr_frame.scan.timestamp] = w_X_lh;

        prev_frame = curr_frame;
        voxel._voxel_map.clear();
    }
}

std::vector<SCalibrationStep> collectSCalibrationSteps(
    const std::vector<Frame3D> &all_framesS,
    Voxel3D &voxel,
    const Canvas &background_image,
    const Zuppo &zuppo,
    const std::map<uint64_t, Eigen::Isometry2f> &odom_buffer)
{
    std::cout << "Finished collecting localization data. Starting to collect calibration data..." << std::endl;

    // Phase 2: Extract vertical lidar measurements & construct calibration vector
    double start_time = 0;
    bool is_timer_running = false;

    Eigen::Isometry2f r_X_lh_inv = iso3dTo2d(zuppo._lidar_vec[HLIDAR_IDX_FOR_ODOMETRY_CALIBRATION]._r_X_l).inverse();

    SCalibrationStep step;

    std::vector<SCalibrationStep> calibration_vector;

    Canvas back_image_temp;
    background_image.copyTo(back_image_temp);

    for (size_t i = 1 + FRAME_START_IDX; i < all_framesS.size(); ++i)
    {
        printProgressStats(i, all_framesS.size(), start_time, is_timer_running);

        const auto &curr_frame = all_framesS[i];
        const uint64_t ts = curr_frame.scan.timestamp;
        Eigen::Isometry2f w_T_lh = odomAt(odom_buffer, ts);

        step.w_T_r = iso2dTo3d(w_T_lh * r_X_lh_inv);

        for (const auto &z : curr_frame.scan.points)
        {
            voxel.addPoint(z);
        }
        const Eigen::Matrix<float, 3, Eigen::Dynamic> zs_voxelV = voxel.extractVoxelMatrix(); //[x,y,z,1]

        step.l_z = zs_voxelV;
        calibration_vector.push_back(std::move(step));
        voxel._voxel_map.clear();
    }

    return calibration_vector;
}

std::vector<Scan3D> assembleDeskewedSScans(
    const LogData &data,
    const std::vector<LidarPacket> &pkts,
    const int lidar_idx,
    const SLidarCalibrator &slidarc,
    const Zuppo &zuppo)
{
    std::vector<Scan3D> scans;
    Scan3D current;
    PacketAssembler assembler;

    int prev_idx = -1;
    uint64_t t_0 = 0;
    ServoEntry s_start;

    const Eigen::Isometry3f r_X_l = slidarc._r_X_l;
    const Eigen::Isometry3f l_X_r = r_X_l.inverse();

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

            const Eigen::Vector3f p_raw(d * std::cos(angle_rad), -d * std::sin(angle_rad), 0);

            const float fraction = j / 11.0f;
            const Eigen::Vector2f interpolated_ticks =
                ticks_at_packet_start.cast<float>() +
                fraction * (ticks_at_packet_end - ticks_at_packet_start).cast<float>();

            const Eigen::Isometry3f dl_isometry = l_X_r * iso2dTo3d(zuppo.hOdom(Eigen::Vector3f(zuppo._kl, zuppo._kr, zuppo._wb), interpolated_ticks)) * r_X_l;
            // std::cerr<< iso2dTo3d(zuppo.hOdom(Eigen::Vector3f(zuppo._kl, zuppo._kr, zuppo._wb), interpolated_ticks)).matrix()<<std::endl;

            const Eigen::Vector3f p_deskewed = dl_isometry * p_raw;
            // std::cerr<< p_deskewed.transpose()<<std::endl;
            current.points.push_back(p_deskewed);
        }
    }

    if (!current.points.empty())
    {
        scans.push_back(std::move(current));
    }

    return scans;
}

std::vector<Frame3D> assembleDeskewedSFrames(const LogData &data, uint8_t lidar_idx, const SLidarCalibrator &slidarc, const Zuppo &zuppo)
{
    PacketAssembler assembler;

    auto scans = assembleDeskewedSScans(data, data.lidar, lidar_idx, slidarc, zuppo);
    std::vector<Frame3D> frames;
    frames.reserve(scans.size());
    for (auto &scan : scans)
    {
        Frame3D f;
        f.scan = std::move(scan);
        // f.odom = assembler.nearestOdom(data.odom, f.scan.timestamp);
        f.servo = assembler.nearestServo(data.servo, f.scan.timestamp);
        frames.push_back(std::move(f));
    }
    return frames;
}