#pragma once

#include "geometry_helpers.h"
#include "calibrator.h"

/** @brief Calibrates horizontal LiDAR pose and robot odometry parameters (6-DOF state)*/
class HLidarCalibrator : public Calibrator<6>
{
public:
  /**
   * @brief Constructs an HLidarCalibrator instance initializing parameters from Zuppo
   * @param zuppo Reference to Zuppo
   * @param lidar_idx Horizontal LiDAR idx
   * @param max_error Maximum error threshold for a point to be an inlier
   * @param kernel_threshold Threshold for downweighting points with errors that exceed acceptable limits yet remain below max_error.
   * @param damping Damping parameter used to stabilize the iterative Gauss-Newton system update.
   */
  HLidarCalibrator(
      Zuppo &zuppo,
      const uint8_t lidar_idx,
      const float max_error = 0.1f,
      const float kernel_threshold = 0.001f,
      const float damping = 0.3f);

  /**
   * @brief Runs the iterative Gauss-Newton optimization loop and updates calibrated parameters in Zuppo.
   *
   * @param steps Vector of calibration steps
   * @param max_iterations Maximum allowed optimization iterations.
   * @return std::vector<float> Vector containing convergence metrics or chi-squared error history.
   */
  std::vector<float> calibrate(const std::vector<HCalibrationStep> &steps, const size_t max_iterations);

  /**
   * @brief Assembles the current 6D state vector from odometry parameters and LiDAR pose.
   *
   * @return StateVector Packed 6D vector containing `[kl, kr, wb, x, y, theta]`.
   */
  inline StateVector getState() const
  {
    Eigen::Vector3f pose(t2v(_r_X_l));
    StateVector x;
    x << _kl, _kr, _wb, pose.x(), pose.y(), pose.z();
    return x;
  }

  /**
   * @brief Gaussian-Newton step
   *
   * @param x Current state vector `[kl, kr, wb, x, y, theta]`.
   * @param step Calibration
   */
  void addMeasurement(const StateVector &x, const HCalibrationStep &step);

  /**
   *@brief Updates the pose estimate after Gauss-Newton optimization
   *
   * @param x State vector to be updated.
   */
  void update(StateVector &x);

  /**
   * @brief @brief Applies an increment to the input state
   *
   * @param x Current 6D state vector.
   * @param increment The 6D delta vector to add to the state.
   * @return StateVector Updated 6D state vector.
   */
  inline StateVector boxplus(const StateVector &x, const StateVector &increment) const
  {
    Eigen::Vector3f v = x.head<3>() + increment.head<3>();
    Eigen::Vector3f T = t2v(v2t<float>(increment.tail<3>()) * v2t<float>(x.tail<3>()));
    return StateVector(v.x(), v.y(), v.z(), T.x(), T.y(), T.z());
  }

  /**
   * @brief Computes the manifold difference vector between two 2D isometry transformations (`T2⁻¹ * T1`).
   *
   * @param T1 First 2D rigid transformation (Isometry).
   * @param T2 Second 2D rigid transformation (Isometry).
   * @return Eigen::Vector3f Relative 2D transformation represented as `[dx, dy, dtheta]`.
   */
  inline Eigen::Vector3f boxminus(const Eigen::Isometry2f &T1, const Eigen::Isometry2f &T2) const
  {
    return t2v(T2.inverse() * T1);
  }

  /**
   * @brief Computes predicted relative LiDAR transformation from odometry ticks
   *
   * @param x Current 6D state vector containing odometry and sensor calibration parameters.
   * @param ticks Accumulated left and right encoder ticks `[left_ticks, right_ticks]`.
   * @return Eigen::Isometry2f Predicted 2D relative motion in the sensor frame.
   */
  inline Eigen::Isometry2f predFunction(const StateVector &x, const Eigen::Vector2f &ticks) const
  {
    Eigen::Isometry2f r_X_l = v2t<float>(x.tail<3>());
    return r_X_l.inverse() * _zuppo.hOdom(x.head<3>(), ticks) * r_X_l;
  }

  float &_kl;               ///< Left wheel coefficient
  float &_kr;               ///< Right wheel coefficient
  float &_wb;               ///< Wheelbase (distance between wheels)
  Eigen::Isometry2f _r_X_l; ///< LiDAR pose w.r.t. the robot
  const uint8_t _lidar_idx; ///< LiDAR idx
  const float _damping;     ///< Damping parameter used to stabilize the iterative Gauss-Newton system update.
};