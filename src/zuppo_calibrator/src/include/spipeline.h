#pragma once

#include <vector>
#include "structs.h"
#include "grid_map.h"
#include "dmap_localizer.h"
#include "voxel.h"
#include "zuppo.h"
#include "slidar_calibrator.h"
#include "display_utils.h"
#include "params.h"
#include "helpers.h"
#include "geometry_helpers.h"
#include "plot.h"
#include "voxel3d.h"

/// SLANTED

/**
 * @brief Replay the trajectory and save localized pose with timestamp. This is needed to interpolate in the next steps
 * @param all_framesH      Vector of recorded horizontal LiDAR frames.
 * @param gmap             GridMap
 * @param loc              DMapLocalizer
 * @param voxel            Voxel
 * @param background_image Canvas displaying background maps for visualization
 * @param plotting         Boolean flag to enable or disable plotting during collection
 * @param loc_history      Output vector accumulating localized positions
 * @param zuppo            Zuppo
 * @param odom_buffer      Output map storing timestamp-to-isometry (`Eigen::Isometry2f`) localized poses.
 * @param w_X_l0           Initial LiDAR pose transformation in the world frame
 */
void collectLocalizationPoses(
    const std::vector<Frame> &all_framesH,
    DMapLocalizer &loc,
    Voxel &voxel,
    const Canvas &background_image,
    std::vector<Eigen::Vector2f> &loc_history,
    const Zuppo &zuppo,
    std::map<uint64_t, Eigen::Isometry2f> &odom_buffer,
    const Eigen::Isometry2f &w_X_l0);

/**
 * @brief Collects valid calibration steps for slanted LiDAR
 *
 * @param all_framesS Vector of recorded slanted LiDAR Frames
 * @param gmap GridMap
 * @param loc DMapLocalizer
 * @param slidarc SLidarCalibrator
 * @param voxel Voxel
 * @param background_image Canvas displaying background maps for visualization
 * @param plotting Boolean flag to enable or disable plotting during collection
 * @param odom_history Output vector accumulating odometry positions
 * @param loc_history Output vector accumulating localized positions
 * @param zuppo Zuppo
 * @param w_X_l0 Initial LiDAR pose transformation in the world frame
 * @return std::vector<SCalibrationStep> A collection of valid calibration steps
 */
std::vector<SCalibrationStep> collectSCalibrationSteps(
    const std::vector<Frame3D> &all_framesS,
    Voxel3D &voxel,
    const Canvas &background_image,
    const Zuppo &zuppo,
    const std::map<uint64_t, Eigen::Isometry2f> &odom_buffer);

/**
 * @brief Assembles all scans from data by deskewing packets points.
 * @param data Full dataset coming from LogData()
 * @param pkts Collection of raw LiDAR packets to process.
 * @param lidar_idx LiDAR idx
 * @param slidarc SLidarCalibrator
 * @param zuppo Zuppo
 * @return Vector array containing paired data.
 */
std::vector<Scan3D> assembleDeskewedSScans(
    const LogData &data,
    const std::vector<LidarPacket> &pkts,
    const int lidar_idx,
    const SLidarCalibrator &slidarc,
    const Zuppo &zuppo);

/**
 * @brief Assembles all frames from data after deskewing the points with assembleDeskewedSScans(.). A frame will have Timestamp matching scans and servo entries.
 * @param data Full dataset coming from LogData()
 * @param lidar_idx LiDAR idx
 * @param slidarc SLidarCalibrator
 * @param zuppo Zuppo
 * @return std::vector<Frame> Vector of fully deskewed frames containing matched scans and servo entries.
 */
std::vector<Frame3D> assembleDeskewedSFrames(const LogData &data, uint8_t lidar_idx, const SLidarCalibrator &slidarc, const Zuppo &zuppo);
