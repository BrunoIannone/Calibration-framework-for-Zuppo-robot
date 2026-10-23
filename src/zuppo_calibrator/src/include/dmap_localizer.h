#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "distance_map.h"

/** @brief Localize the robot on a 2D distance map */
struct DMapLocalizer : public DistanceMap
{

  /** @brief Constructs a DMapLocalizer with the given parameters
   * @param dmap DistanceMap to localize on
   * @param max_range The maximum obstacle accepted distance (meters).
   * @param use_bilinear Whether to use bilinear interpolation for distance and gradient fetching.
   * @param point_damping Damping factor per point.
   * @param kernel_threshold Threshold for downweighting points with errors that exceed acceptable limits yet remain below max_error.
   * @param max_error Maximum error threshold for a point to be an inlier
   */
  DMapLocalizer(const DistanceMap &dmap, float max_range,
                bool use_bilinear = true, float point_damping = 1e-2, float kernel_threshold = 0.5, float max_error = 0.1);

  /**
   * @brief Fetches the distance and gradients at a given point.
   * @param distance The distance value from the point on the grid to its closest obstacle.
   * @param gradients The gradients vector [drow,dcols].
   * @param pg The point on the grid (in float values).
   * @return True if the point is inside the grid, false otherwise.
   */
  bool fetchDistanceAndGradients(float &distance,
                                 Eigen::Vector2f &gradients,
                                 const Eigen::Vector2f &pg) const;

  /** @brief Resets the Gaussian-Newton optimization.
   */
  void reset();

  /** @brief Gaussian-Newton step.
   * @param l_z The measurement point in lidar frame
   */
  void addMeasurement(const Eigen::Vector2f &l_z);

  /**
   * @brief Updates the pose estimate after Gauss-Newton optimization.
   * This function should be called after using addMeasurement().
   */
  void update();

  /**
   * @brief Start the Gauss-Newton loop
   * @param l_z Lidar 2D points in LiDAR frame
   * @param max_iterations Maximum number of solver iterations (default: 10).
   */
  void localize(const std::vector<Eigen::Vector2f> &l_z, const int max_iterations = 10);

  // MEMBER VARIABLES
  float _chi2;
  float _point_damping;
  float _kernel_threshold;
  bool _use_bilinear;
  float _max_error;

  float _total_damping;
  uint32_t _num_inliers;
  uint32_t _num_outliers;

  Eigen::Matrix3f H;
  Eigen::Vector3f b;
  Eigen::Vector3f dx;  ///< Computed increment
  Eigen::Isometry2f X; ///< The pose in the world being optimized

  Grid_<float> _distances;
  Grid_<float>::GridGradientType _distance_gradients;
};
