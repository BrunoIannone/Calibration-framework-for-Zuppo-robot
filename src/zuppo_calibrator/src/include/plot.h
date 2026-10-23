#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief Render Chi2 per iteration
 * @param history Vector of Chi2 values per iteration
 */
void plotChi(const std::vector<float> &history);

/**
 * @brief Plot 2D trajectory
 * @param odom_history Vector of 2D pose coordinates
 * @param label_id Legend display name for the trajectory line. Default to "Odometry"
 */
void plotTrajectory(const std::vector<Eigen::Vector2f> &odom_history, const char *label_id = "Odometry");

/**
 * @brief If given, renders the plot of Chi2, odometry and localized/calibrated trajectories.
 * @param odom_history Raw odometry trajectory points
 * @param cal_odom_history Localized/calibrated trajectory points
 * @param history Vector of Chi2 values per iteration
 */
void renderPlot(const std::vector<Eigen::Vector2f> &odom_history = {}, const std::vector<Eigen::Vector2f> &cal_odom_history = {}, const std::vector<float> &history = {});

/**
 * @brief Render 2D map coloring walls (red) and floor (blue) points
 * @param calibration_vector Vector of calibration steps containing scan points
 * @param r_X_l Relative pose of the LiDAR w.r.t. the robot
 * @param gmap Grid map
 */
void plotFloorAndWall2D(Canvas colored_bg,
                        const std::vector<SCalibrationStep> &calibration_vector,
                        const Eigen::Isometry3f &r_X_l,
                        const GridMap &gmap);

/**
 * @brief Render 3D point cloud coloring walls (red) and floor (blue) points
 * @param calibration_vector Vector of processed calibration steps containing scan points
 * @param r_X_l Relative pose of the LiDAR w.r.t. the robot
 * @param gmap Grid map
 */
void plotFloorAndWall3D(
    const std::vector<SCalibrationStep> &calibration_vector,
    const Eigen::Isometry3f &r_X_l,
    const GridMap &gmap);
