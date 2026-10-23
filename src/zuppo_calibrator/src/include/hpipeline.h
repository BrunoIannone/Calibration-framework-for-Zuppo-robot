#pragma once
#include <Eigen/Core>
#include "structs.h"
#include "dmap_localizer.h"
#include "hlidar_calibrator.h"
#include "voxel.h"

/**
 * @brief Collects valid calibration steps for horizontal LiDAR and odometry calibration from recorded data frames.
 *
 * @param all_framesH Vector of recorded horizontal LiDAR Frames
 * @param gmap GridMap
 * @param loc DMapLocalizer
 * @param hlidarc HLidarCalibrator
 * @param voxel Voxel
 * @param background_image Canvas displaying background maps for visualization
 * @param plotting Boolean flag to enable or disable plotting during collection
 * @param odom_history Output vector accumulating odometry positions
 * @param loc_history Output vector accumulating localized positions
 * @param zuppo Zuppo
 * @param w_X_l0 Initial LiDAR pose transformation in the world frame
 * @return std::vector<HCalibrationStep> A collection of valid calibration steps
 */
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
    const Eigen::Isometry2f &w_X_l0);

/**
 * @brief Replays a trajectory using current parameters
 *
 * @param all_framesH Vector of recorded horizontal LiDAR frames
 * @param hlidarc HLidarCalibrator
 * @param odom_history Output vector accumulating odometry positions
 * @param zuppo Zuppo
 * @param w_X_l0 Initial LiDAR pose transformation in the world frame
 */
void replayTrajectory(
    const std::vector<Frame> &all_framesH,
    const HLidarCalibrator &hlidarc,
    std::vector<Eigen::Vector2f> &odom_history,
    const Zuppo &zuppo,
    const Eigen::Isometry2f &w_X_l0);

/**
 * @brief Assembles all scans from data by deskewing packets points.
 * @param data Full dataset coming from LogData()
 * @param pkts Collection of raw LiDAR packets to process.
 * @param lidar_idx LiDAR idx
 * @param hlidarc HLidarCalibrator
 * @param zuppo Zuppo
 * @return Vector array containing paired data.
 */
std::vector<Scan> assembleDeskewedScans(
    const LogData &data,
    const std::vector<LidarPacket> &pkts,
    const int lidar_idx,
    const HLidarCalibrator &hlidarc,
    const Zuppo &zuppo);

/**
 * @brief Assembles all frames from data after deskewing the points with assembleDeskewedScans(.). A frame will have Timestamp matching scans and servo entries.
 * @param data Full dataset coming from LogData()
 * @param lidar_idx LiDAR idx
 * @param hlidarc HLidarCalibrator
 * @param zuppo Zuppo
 * @return std::vector<Frame> Vector of fully deskewed frames containing matched scans and servo entries.
 */
std::vector<Frame> assembleDeskewedHFrames(const LogData &data, uint8_t lidar_idx, const HLidarCalibrator &hlidarc, const Zuppo &zuppo);