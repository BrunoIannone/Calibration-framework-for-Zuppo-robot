#pragma once

#include "geometry_helpers.h"
#include "zuppo.h"
#include "calibrator.h"
#include "params.h"
#include "structs.h"
#include "grid_map.h"
#include "dmap_localizer.h"

/**
 * @brief Calibrates slanted LiDAR pose (5-DOF state)
 */
class SLidarCalibrator : public Calibrator<5>
{
public:
    /**
     * @brief Constructs a SLidarCalibrator instance.
     * @param zuppo Zuppo
     * @param lidar_idx Slanted LiDAR idx
     * @param gmap GridMap
     * @param loc DMapLocalizer
     * @param max_error Maximum error threshold for a point to be an inlier
     * @param kernel_threshold hreshold for downweighting points with errors that exceed acceptable limits yet remain below max_error.
     * @param point_damping Damping factor per point
     */
    SLidarCalibrator(
        Zuppo &zuppo,
        const uint8_t lidar_idx,
        const GridMap &gmap,
        const DMapLocalizer &loc,
        const float max_error,
        const float kernel_threshold,
        const float point_damping)

        : Calibrator<5>(zuppo, max_error, kernel_threshold),
          _r_X_l(_zuppo._lidar_vec[lidar_idx]._r_X_l),
          _point_damping(point_damping),
          _loc(loc), _gmap(gmap)
    {
    }

    /**
     * @brief Runs the iterative Gauss-Newton optimization
     * @param steps Vector of calibration steps
     * @param max_iterations Maximum allowed optimization iterations.
     * @return std::vector<float> Vector containing convergence metrics or chi-squared error history.
     */
    std::vector<float> calibrate(const std::vector<SCalibrationStep> &steps, const size_t max_iterations);

    /**
     *@brief Updates the pose estimate after Gauss-Newton optimization
     */
    void update();

    /**
     * @brief Gaussian-Newton step
     *
     * @param robot_pose Current 3D pose matrix of the robot platform in world frame.
     * @param z 3D point in LiDAR frame.
     */
    void addMeasurement(const Eigen::Isometry3f &robot_pose, const Eigen::Vector3f &l_z);

    /**
     * @brief Resets the linear system accumulators and optimization metrics to zero.
     */
    inline void reset() override
    {
        H.setZero();
        b.setZero();
        dx.setZero();
        _chi2 = 0.0f;
        _num_inliers = 0;
        _num_outliers = 0;
        _total_damping = 0;
    }
    /**
     * @brief Builds a 3D rigid isometry transformation matrix from 5D state delta vector.
     * @param x 5D delta vector (2D translation + 3D orientation Euler angles).
     * @param z0 Z-axis offset value.
     * @return 3D Rigid transform representing the delta pose.
     */
    inline Eigen::Isometry3f buildIsometryFromState(const StateVector &x, float z0) const
    {
        Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
        iso.translation() << x.head<2>(), z0;
        iso.linear() = angles2R(x.tail<3>());
        return iso;
    }

    Eigen::Isometry3f &_r_X_l; ///< LiDAR pose w.r.t. the robot

protected:
    int n_floor = 0;
    int n_wall = 0;

    const float _point_damping;
    const float _z_floor_max = Z_FLOOR_MAX;
    const float _z_floor_min = Z_FLOOR_MIN;

    const float _z_wall_max = Z_WALL_MAX;
    const float _z_wall_min = Z_WALL_MIN;

    float _total_damping;
    float _w_wall = 1.0f;
    float _w_floor = 1.0f;
    const double _z_0 = 0.15; ///< Z-axis offset value.

    const DMapLocalizer &_loc;
    const GridMap &_gmap;
};