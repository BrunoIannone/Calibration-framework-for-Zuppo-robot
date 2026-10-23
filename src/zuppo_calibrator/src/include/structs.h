#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "params.h"

using Vector6f = Eigen::Matrix<float, 6, 1>;

// ============================================================================
// Structs for log data
// ============================================================================

/**
 * @brief Struct that contains all data of a lidar packet
 */
struct LidarPacket
{
  uint64_t timestamp;
  uint16_t idx;         ///< idx of the lidar producing the packet
  uint16_t start_angle; ///< Hundredths of degrees (0-36000)
  uint16_t end_angle;   ///< Hundredths of degrees (0-36000)
  uint16_t speed;       ///< Hundredths of deg/s
  /**
   * @brief Each packet contains PACKET_SIZE number of points. Each with its confidence score
   */
  struct
  {
    uint16_t distance;
    int confidence;
  } points[PACKET_SIZE];
};

/**
 * @brief Struct that contains all data of an odometry entry
 */
struct OdomEntry
{
  uint64_t timestamp;
  float x, y, theta;
  float tv; ///< Translational velocity
  float rv; ///< Rotational velocity
};

/**
 * @brief Struct that contains all data of a servo entry
 */
struct ServoEntry
{
  uint64_t timestamp;
  uint32_t ticks[2]{0, 0}; ///< Left and right tick array
};

/**
 * @brief A full LiDAR scan assembled from consecutive packets
 */
struct Scan
{
  uint64_t timestamp;
  std::vector<Eigen::Vector2f> points; ///< Cartesian 2D points in LiDAR frame (meters)
};

/**
 * @brief Struct that contains the LiDAR pose in quaternion format
 */
struct LidarConfig
{
  float tx = 0, ty = 0, tz = 0, qx = 0, qy = 0, qz = 0, qw = 1;
};

/**
 * @brief Struct that contains the robot parameters
 */
struct UnicycleConfig
{

  float kl;     ///< Left wheel coefficient
  float kr;     ///< Right wheel coefficient
  float wb;     ///< Wheelbase (distance between wheels)
  float tv_max; ///< Maximum translational velocity
  float rv_max; ///< Maximum rotational velocity
  float ta_max; ///< Maximum translational acceleration
  float ra_max; ///< Maximum rotational acceleration
};

/**
 * @brief Struct that contains the full parsed dataset
 */
struct LogData
{
  LogData() {};
  std::vector<OdomEntry> odom;
  std::vector<LidarPacket> lidar;
  std::vector<LidarConfig> lidar_configs;
  std::vector<ServoEntry> servo;
  UnicycleConfig unicycle_config;
};

/**
 * @brief Struct that contains the all the data for a frame. A frame is composed by a 360° scan and the related servo entry (according to closest timestamp)
 */
struct Frame
{
  Frame() {};
  Scan scan;
  // OdomEntry odom;
  ServoEntry servo;
};

/**
 * @brief A full LiDAR scan assembled from consecutive 3D packets
 */
struct Scan3D
{
  uint64_t timestamp;
  std::vector<Eigen::Vector3f> points; ///< Cartesian 2D points in LiDAR frame (meters)
};

/**
 * @brief Struct that contains the all the data for a 3D frame. A frame is composed by a 360° scan and the related servo entry (according to closest timestamp)
 */
struct Frame3D
{
  Frame3D() {};
  Scan3D scan;
  // OdomEntry odom;
  ServoEntry servo;
};
// ============================================================================
// Zuppo
// ============================================================================

/**
 * @brief Struct that contains mounted LiDAR data
 */
struct Lidar // Keep for now, might be useful to extend it later
{
  /**
   * @brief Lidar constructor
   * @param _r_X_l LiDAR pose w.r.t. the robot
   */
  Lidar(const Eigen::Isometry3f &r_X_l) : _r_X_l(r_X_l) {}
  Lidar() {}
  Eigen::Isometry3f _r_X_l; ///< LiDAR pose w.r.t. the robot
};

// ============================================================================
// HLidar Calibration
// ============================================================================

/**
 * @brief Struct containing all data of a valid calibration step for the horizontal LiDAR
 */
struct HCalibrationStep
{
  Eigen::Isometry2f T_a; ///< Initial LiDAR 2D world pose for the current step
  Eigen::Isometry2f T_b; ///< Final LiDAR 2D world pose for the current step
  Eigen::Vector2f ticks; ///< Total ticks from T_a to T_b
};

/**
 * @brief Manages temporal chunking and online movement statistics for horizontal LiDAR calibration
 */
struct Chunk
{
  /**
   * @brief Default constructor initializing the chunk state
   */
  Chunk() { reset(); }

  /**
   * @brief Accumulates delta motion to determine if a criterion is reached and an HCalibration step can be closed and accepted
   * @param T_a The starting 2D world pose for the current step candidate
   * @param T_b The current ending 2D world pose
   * @param ticks Encoder ticks accumulated during this motion increment
   * @param dl Relative pose delta (dx, dy, dtheta)
   * @return true if accumulated motion exceeds acceptance thresholds, false otherwise
   */
  bool addCs(const Eigen::Isometry2f &T_a,
             const Eigen::Isometry2f &T_b,
             const Eigen::Vector2f &ticks,
             const Eigen::Vector3f &dl)
  {
    if (!_hasFirstPose)
    {
      _step.T_a = T_a;
      _hasFirstPose = true;
    }

    _acc_translation += dl.head<2>().norm();
    _acc_rotation += std::abs(dl.z());
    _total_ticks += ticks;

    const bool chunkComplete = (_acc_translation > kTranslationThreshold) ||
                               (_acc_rotation > kRotationThreshold);

    if (chunkComplete)
    {
      _step.T_b = T_b;
      _step.ticks = _total_ticks;
    }

    return chunkComplete;
  }

  /**
   * @brief Computes instant wheel velocities and updates running mean/variance statistics
   * @param x Calibration state vector containing scale factors
   * @param dt Time difference in microseconds
   * @param ticks Encoder ticks recorded during the time difference
   * @return Updated variance vector of wheel velocities
   */
  Eigen::Vector2f addVelocity(const Vector6f &x,
                              uint64_t dt,
                              const Eigen::Vector2f &ticks)
  {
    if (dt == 0)
      return Eigen::Vector2f::Zero();

    _currVel = (x.head<2>().array() * ticks.array() /
                (static_cast<float>(dt) * kTimeScale))
                   .matrix();

    if (std::abs(_currVel.x()) < MIN_VELOCITY &&
        std::abs(_currVel.y()) < MIN_VELOCITY)
    {
      return Eigen::Vector2f::Zero();
    }

    // Online mean/variance update: Var[x] = E[x^2] - E[x]^2.
    _sum += _currVel;
    _sum2 += _currVel.array().square().matrix();
    ++_counter;

    const float invCount = 1.0f / static_cast<float>(_counter);
    const Eigen::Array2f mean = _sum.array() * invCount;
    const Eigen::Array2f mean2 = _sum2.array() * invCount;

    _variance = (mean2 - mean.square()).matrix();

    return _variance;
  }

  /**
   * @brief Resets all accumulated statistics and parameters to their initial zero states
   */
  inline void reset()
  {
    _sum.setZero();
    _sum2.setZero();
    _counter = 0;
    _variance.setZero();
    _currVel.setZero();
    _total_ticks.setZero();
    _acc_translation = 0.0f;
    _acc_rotation = 0.0f;
    _hasFirstPose = false;
  }

  static constexpr float kTranslationThreshold = 0.20f;  ///< Translation distance limit to create a chunk (meters)
  static constexpr float kRotationThreshold = 0.174533f; ///< Rotation angle limit to create a chunk (radians, ~10 deg)
  static constexpr float kTimeScale = 1.0f / 1000000.0f; ///< Conversion factor from microseconds to seconds

  bool _hasFirstPose = false;

  float _acc_translation = 0.0f;
  float _acc_rotation = 0.0f;

  Eigen::Vector2f _total_ticks = Eigen::Vector2f::Zero();

  // Running velocity statistics.
  size_t _counter = 0;
  Eigen::Vector2f _sum = Eigen::Vector2f::Zero();  ///< Running sum of wheel velocities
  Eigen::Vector2f _sum2 = Eigen::Vector2f::Zero(); ///< Running sum of squared wheel velocities
  Eigen::Vector2f _variance = Eigen::Vector2f::Zero();
  Eigen::Vector2f _currVel = Eigen::Vector2f::Zero();

  HCalibrationStep _step;
};

// ============================================================================
// SLidar Calibration
// ============================================================================

/**
 * @brief Struct containing data for a single spherical/spinning LiDAR calibration step
 */
struct SCalibrationStep
{
  Eigen::Isometry3f w_T_r;                     ///< 3D transformation matrix representing robot pose in world frame
  Eigen::Matrix<float, 3, Eigen::Dynamic> l_z; ///< Matrix of 3D point
};
