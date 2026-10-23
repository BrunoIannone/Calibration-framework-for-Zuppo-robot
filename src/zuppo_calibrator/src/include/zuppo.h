#pragma once
#include "structs.h"
#include <Eigen/Core>
#include "vector"
#include "params.h"
#include "geometry_helpers.h"

/**
 * @brief Class for Zuppo robot
 */

struct Zuppo
{
    /**
     * @brief Zuppo constructor
     * @param n_lidar Number of mounted LiDARs
     * @param cfg Unicycle initial parameters
     * @param lidar_T Vector of lidar Isometries
     */
    Zuppo(const uint8_t n_lidar, const UnicycleConfig &cfg, const std::vector<Eigen::Isometry3f> &lidar_T) : _kl(cfg.kl), _kr(cfg.kr), _wb(cfg.wb)
    {
        _lidar_vec.resize(n_lidar);
        for (uint8_t i = 0; i < _lidar_vec.size(); ++i)
        {
            if (LIDAR_STATUS[i] < 0)
            {
                std::cerr << "ZUPPO: " << RED << "Unmounted LiDAR detected in position: " << RESET << i << std::endl;
            }

            _lidar_vec[i] = Lidar(lidar_T[i]);
        }
    }

    inline Eigen::Isometry2f hOdom(const Eigen::Vector3f &x, const Eigen::Vector2f &ticks) const;

    std::vector<Lidar> _lidar_vec; ///< Vector of LiDAR poses w.r.t. the robot
    float _kl;                     ///< Left wheel coefficient
    float _kr;                     ///< Right wheel coefficient
    float _wb;                     ///< Wheelbase (distance between wheels)
};

/**
 * @brief Computes odometry increment
 * @param x Zuppo's odometry vector [kl,kr,wb]
 * @param ticks Left and Right ticks vector
 * @return Increment pose
 */
inline Eigen::Isometry2f Zuppo::hOdom(const Eigen::Vector3f &x, const Eigen::Vector2f &ticks) const
{
    const float tl = ticks.x();
    const float tr = ticks.y();
    if (tl == 0.0f && tr == 0.0f)
    {
        return Eigen::Isometry2f::Identity();
    }

    const float dl = tl * x.x();
    const float dr = tr * x.y();
    const float dth = ((dr - dl) / x.z());
    const float d_plus2 = 0.5f * (dr + dl);

    float dx = 0.0f;
    float dy = 0.0f;

    const float sd = std::sin(dth), cd = std::cos(dth);

    if (std::abs(dth) < 1e-4f)
    {
        dx = d_plus2;
        dy = 0.0f;
    }
    else
    {
        const float inv_dth = 1.0f / dth;
        dx = d_plus2 * sd * inv_dth;
        dy = d_plus2 * (1.0f - cd) * inv_dth;
    }

    Eigen::Isometry2f T = Eigen::Isometry2f::Identity();

    T.linear() << cd, -sd, sd, cd;
    T.translation() << dx, dy;
    return T;
}