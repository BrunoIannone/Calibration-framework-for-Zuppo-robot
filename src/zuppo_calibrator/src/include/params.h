#pragma once
#include <iostream>
#include <cstdint>

constexpr int FRAME_START_IDX = 100; ///< Frame from which the robot starts moving
constexpr int MAX_DT = 300001;       ///< Max allowed time difference
// constexpr int INLIERS_INCREMENT = 25;
constexpr float VARIANCE_THRESHOLD = 0.03 * 0.03; ///< [m^2/s^2] Threshold after which the Chunk is discarded
inline uint32_t MIN_INLIERS = 400;                ///< Minimum number of inliers to consider the pose valid for the calibration data collection

// ============================================================================
// Terminal colors
// ============================================================================

const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string RESET = "\033[0m";
const std::string YELLOW = "\033[33m";

// ============================================================================
// Voxelizer Params
// ============================================================================

constexpr float VOXEL_RESOLUTION = 0.03f; ///< Resolution for the voxelization

// ============================================================================
// DMap Localizer Params
// ============================================================================

constexpr float MAX_RANGE = 10.0f;             ///< The maximum obstacle accepted distance (meters).
constexpr bool USE_BILINEAR = true;            ///< Whether to use bilinear interpolation for distance and gradient fetching.
constexpr float LOCALIZE_POINT_DAMPING = 0.1f; ///< Damping factor per point
constexpr float KERNEL_THRESHOLD = 0.001f;     ///< Threshold for downweighting points with errors that exceed acceptable limits yet remain below max_error.
constexpr float LOCALIZE_MAX_ERROR = 0.1f;     ///< Maximum error threshold for a point to be an inlier
constexpr uint32_t LOCALIZE_MAX_ITERATIONS = 100;

// ============================================================================
// HLidar Calibration Params
// ============================================================================

constexpr float CALIBRATION_MAX_ERROR = 0.1f;          ///< Maximum error threshold for a point to be an inlier
constexpr float CALIBRATION_KERNEL_THRESHOLD = 0.001f; ///< Threshold for downweighting points with errors that exceed acceptable limits yet remain below max_error.
constexpr uint32_t CALIBRATE_MAX_ITERATIONS = 20;
constexpr float CALIBRATION_DAMPING = 0.3; ///< Damping factor per point

// ============================================================================
// SLidar Calibration Params
// ============================================================================

constexpr float SCALIBRATION_MAX_ERROR = 0.2f;         ///< Maximum error threshold for a point to be an inlier
constexpr float SCALIBRATION_KERNEL_THRESHOLD = 0.02f; ///< Threshold for downweighting points with errors that exceed acceptable limits yet remain below max_error.
constexpr uint32_t SCALIBRATE_MAX_ITERATIONS = 30;
constexpr float SCALIBRATION_DAMPING = 0.001f; ///< Damping factor per point

// ============================================================================
// ZUPPO
// ============================================================================

constexpr uint8_t LIDAR_NUM = 2;                   ///< Number of mounted LiDARS
constexpr int8_t LIDAR_STATUS[4] = {1, 1, -1, -1}; ///< Mounted LiDARs map

constexpr int PACKET_SIZE = 12; ///< Number of points per LiDAR packet

constexpr uint8_t HLIDAR_IDX_FOR_ODOMETRY_CALIBRATION = 0; ///< Horizontal LiDAR idx
constexpr uint8_t SLIDAR_IDX_FOR_CALIBRATION = 1;          ///< Slanted LiDAR idx

constexpr uint8_t LEFT_WHEEL_IDX = 1;  ///< Left wheel idx
constexpr uint8_t RIGHT_WHEEL_IDX = 0; ///< Right  wheel idx
constexpr float MIN_VELOCITY = 1e-2f;  ///< Threshold of velocity under which we assume no motion

// ============================================================================
// ENVIRONMENT
// ============================================================================

constexpr float Z_WALL_MIN = 0.5f;   ///< Minimum height for wall points
constexpr float Z_WALL_MAX = 2.0f;   ///< Maximum height for wall points
constexpr float Z_FLOOR_MIN = -0.1f; ///< Minimum height for floor points
constexpr float Z_FLOOR_MAX = 0.1f;  ///< Maximum height for floor points