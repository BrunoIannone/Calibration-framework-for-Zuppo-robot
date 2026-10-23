#pragma once

#include <cmath>
#include <vector>
#include <ankerl/unordered_dense.h>
#include "Eigen/Core"

/**
 * @brief Type alias for 64-bit spatial hash keys combining 2D grid coordinates.
 */
using Key = std::uint64_t;

/**
 * @brief Combines signed 2D grid indices into a unique 64-bit spatial hash key.
 *
 * Packs the two 32-bit integer grid indices into a single 64-bit unsigned integer
 * by bit-shifting `x` into the upper 32 bits and placing `y` in the lower 32 bits.
 *
 * @param x The integer grid x-coordinate.
 * @param y The integer grid y-coordinate.
 * @return A packed 64-bit spatial key representing the `(x, y)` grid location.
 */
inline Key makeKey(int x, int y)
{
    return (static_cast<Key>(static_cast<uint32_t>(x)) << 32) |
           static_cast<uint32_t>(y);
}
/**
 * @brief Struct that holds the current stats for a cell.
 */
struct Value
{
    /**
     * @brief Updates the current stats of a 2D world point.
     * @param p The 2D world point to be added.
     */
    inline void addPoint(const Eigen::Vector2f &p)
    {
        _sum += p;
        ++_counter;
    }
    /**
     * @brief Computes the mean of the voxel cell.
     * @return The 2D average position vector.
     */
    inline Eigen::Vector2f mean() const
    {

        return _sum / static_cast<float>(_counter);
    }

    // MEMBER VARIABLES
    Eigen::Vector2f _sum = Eigen::Vector2f::Zero(); ///< Total sum within a voxel cell
    size_t _counter = 0;                            ///< Total points within a voxel cell.
};

/**
 * @brief Struct to Voxelize a scan represented as a vector of 2D points (std::vector<Eigen::Vector2f>).
 */
struct Voxel
{
    /**
     * @brief Constructor initializing the voxel grid with a specified resolution.
     * @param resolution The size of each voxel cell in meters (e.g., 0.02 for 2cm).
     * */
    Voxel(float resolution) : _resolution(resolution), _inverse_resolution(1.0 / resolution)
    {
    }

    /**
     * @brief Adds a 2D point to the appropriate voxel cell based on its coordinates.
     * @param p The 2D coordinate vector (meters) to register.
     */
    inline void addPoint(const Eigen::Vector2f &p)
    {
        int grid_x = static_cast<int>(std::floor(p.x() * (_inverse_resolution)));
        int grid_y = static_cast<int>(std::floor(p.y() * (_inverse_resolution)));

        _voxel_map[makeKey(grid_x, grid_y)].addPoint(p);
    }

    /**
     * @brief Voxelize all the given 2D points
     * @return std::vector<Eigen::Vector2f> of voxelized 2D points
     */
    inline std::vector<Eigen::Vector2f> voxelize()
    {
        std::vector<Eigen::Vector2f> zs_voxel;
        for (const auto &[_, value] : _voxel_map)
        {
            zs_voxel.push_back(value.mean());
        }
        return zs_voxel;
    }

    ankerl::unordered_dense::map<Key, Value> _voxel_map;
    float _resolution; ///< Spatial grid cell resolution in meters per cell (m/cell).
    float _inverse_resolution;
};
