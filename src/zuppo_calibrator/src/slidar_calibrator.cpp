#include "slidar_calibrator.h"
#include <iostream>

std::vector<float> SLidarCalibrator::calibrate(const std::vector<SCalibrationStep> &steps, const size_t max_iterations)
{
    std::vector<float> chi_history;
    chi_history.reserve(max_iterations);

    for (size_t iter = 0; iter < max_iterations; ++iter)
    {
        reset();
        for (const auto &step : steps)
        {
            for (int i = 0; i < step.l_z.cols(); ++i)
            {

                addMeasurement(step.w_T_r, step.l_z.col(i));
            }
        }

        update();
        chi_history.push_back(_chi2);
    }

    return chi_history;
}

void SLidarCalibrator::update()
{
    H += Eigen::Matrix<float, 5, 5>::Identity() * 0.001;
    dx = H.ldlt().solve(b);

    Eigen::Isometry3f dX = buildIsometryFromState(dx, 0);
    _r_X_l.linear() = dX.linear() * _r_X_l.linear();

    _r_X_l.translation() += dX.translation();
}

void SLidarCalibrator::addMeasurement(const Eigen::Isometry3f &robot_pose, const Eigen::Vector3f &l_z)
{
    const Eigen::Vector3f p_rot = _r_X_l.linear() * l_z;
    const Eigen::Vector3f w_p = robot_pose * (p_rot + _r_X_l.translation());

    float p_z = w_p.z();

    bool is_wall = (p_z >= _z_wall_min && p_z < _z_wall_max);
    bool is_floor = (p_z >= _z_floor_min && p_z < _z_floor_max);
    if (!is_wall && !is_floor)
    {
        //++_num_outliers;
        return;
    }
    Eigen::Matrix<float, 3, 5> J_pre;
    J_pre.setZero();
    J_pre.block<2, 2>(0, 0).setIdentity();
    J_pre.block<3, 3>(0, 2) = skew<float>(-p_rot);
    const Eigen::Matrix<float, 3, 5> J_icp = robot_pose.linear() * J_pre;

    const Eigen::Vector2f pg = _loc.worldToGridF(w_p.head<2>());
    float e;
    Eigen::Vector2f grad2d;
    Eigen::Vector3f grad3d;
    grad3d.setZero();

    if (!_loc.fetchDistanceAndGradients(e, grad2d, pg))
    {
        ++_num_outliers;
        return;
    }

    if (is_wall)
    {
        // std::cerr << "wall: " << e << std::endl;

        if (e <= _max_error)
        {
            grad3d.head<2>() = grad2d;
        }
        else
        {
            ++_num_outliers;
            return;
        }
    }
    else
    {
        if (e < _max_error)
        {
            ++_num_outliers;
            return;
        }
        const uint8_t value = _gmap.at(pg.y(), pg.x());
        if (value == _gmap.UNKNOWN)
        {
            //++_num_outliers;
            return;
        }
        // else if(value == _gmap.UNKNOWN)
        // {
        //     std::cerr<<"UNKONW"<<(int)value<<std::endl;

        //     ++_num_outliers;
        //     return;
        // }
        grad3d(2) = 1.0f;
        e = p_z;
    }

    _total_damping += _point_damping;
    float this_chi = e * e;
    float kscale = 1.0f;
    if (this_chi > _kernel_threshold)
    {
        kscale = sqrt(_kernel_threshold / this_chi);
        ++_num_outliers;
    }
    else
    {
        ++_num_inliers;
    }
    _chi2 += this_chi * kscale;
    // const float weight = is_wall ? _w_wall : _w_floor;
    //  kscale *= weight;
    const auto temp = grad3d[1];
    grad3d[1] = -grad3d[0];
    grad3d[0] = temp;
    Eigen::Matrix<float, 1, 5> J = grad3d.transpose() * J_icp;
    H.selfadjointView<Eigen::Upper>().rankUpdate(J, kscale);
    b.noalias() -= J * (e * kscale);
}
