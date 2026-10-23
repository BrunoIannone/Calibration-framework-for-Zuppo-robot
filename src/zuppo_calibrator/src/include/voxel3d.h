#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <ankerl/unordered_dense.h>
#include "Eigen/Core"

/**
 * @brief Type alias for 64-bit spatial hash keys combining 3D grid coordinates.
 */
using Key = std::uint64_t;

/**
 * @brief Combines signed 3D grid indices into a unique 64-bit spatial hash key.
 *
 * Packs the three integer grid indices into a single 64-bit unsigned integer,
 * allotting 21 bits per axis (63 bits total). Each signed index is biased by
 * 2^20 before packing so it fits in an unsigned 21-bit field, giving each axis
 * a representable range of roughly [-1,048,576, 1,048,575] grid cells. At a
 * typical 2cm resolution that corresponds to about ±20km per axis, which is
 * far beyond the extent of any real sensor scan.
 *
 * @param x The integer grid x-coordinate.
 * @param y The integer grid y-coordinate.
 * @param z The integer grid z-coordinate.
 * @return A packed 64-bit spatial key representing the `(x, y, z)` grid location.
 */
inline Key makeKey(int x, int y, int z)
{
    constexpr int BITS = 21;
    constexpr std::uint64_t MASK = (static_cast<std::uint64_t>(1) << BITS) - 1;
    constexpr std::int64_t OFFSET = static_cast<std::int64_t>(1) << (BITS - 1);

    const std::uint64_t ux = (static_cast<std::int64_t>(x) + OFFSET) & MASK;
    const std::uint64_t uy = (static_cast<std::int64_t>(y) + OFFSET) & MASK;
    const std::uint64_t uz = (static_cast<std::int64_t>(z) + OFFSET) & MASK;

    return (ux << (2 * BITS)) | (uy << BITS) | uz;
}
/**
 * @brief Struct that holds the current stats for a cell.
 */
struct Value3D
{
    /**
     * @brief Updates the current stats of a 3D world point.
     * @param p The 3D world point to be added.
     */
    inline void addPoint(const Eigen::Vector3f &p)
    {
        _sum += p;
        ++_counter;
    }
    /**
     * @brief Computes the mean of the voxel cell.
     * @return The 3D average position vector.
     */
    inline Eigen::Vector3f mean() const
    {

        return _sum / static_cast<float>(_counter);
    }

    // MEMBER VARIABLES
    Eigen::Vector3f _sum = Eigen::Vector3f::Zero(); ///< Total sum within a voxel cell
    size_t _counter = 0;                            ///< Total points within a voxel cell.
};

/**
 * @brief Struct to Voxelize a scan represented as a vector of 3D points (std::vector<Eigen::Vector3f>).
 */
struct Voxel3D
{
    /**
     * @brief Constructor initializing the voxel grid with a specified resolution.
     * @param resolution The size of each voxel cell in meters (e.g., 0.02 for 2cm).
     * */
    Voxel3D(float resolution) : _resolution(resolution), _inverse_resolution(1.0 / resolution)
    {
    }

    /**
     * @brief Adds a 3D point to the appropriate voxel cell based on its coordinates.
     * @param p The 3D coordinate vector (meters) to register.
     */
    inline void addPoint(const Eigen::Vector3f &p)
    {
        int grid_x = static_cast<int>(std::floor(p.x() * (_inverse_resolution)));
        int grid_y = static_cast<int>(std::floor(p.y() * (_inverse_resolution)));
        int grid_z = static_cast<int>(std::floor(p.z() * (_inverse_resolution)));

        _voxel_map[makeKey(grid_x, grid_y, grid_z)].addPoint(p);
    }

    /**
     * @brief Voxelize all the given 3D points
     * @return std::vector<Eigen::Vector2f> of voxelized 3D points
     */
    inline std::vector<Eigen::Vector3f> voxelize()
    {
        std::vector<Eigen::Vector3f> zs_voxel;
        for (const auto &[_, value] : _voxel_map)
        {
            zs_voxel.push_back(value.mean());
        }
        return zs_voxel;
    }

    /**
     * @brief Voxelize all the given 3D points
     * @return Eigen::Matrix3Xf of voxelized 3D points. Each column is a point
     */
    Eigen::Matrix3Xf extractVoxelMatrix()
    {

        Eigen::Matrix3Xf zs_voxel(3, _voxel_map.size());
        int col = 0;

        for (const auto &[key, value] : _voxel_map)
        {

            zs_voxel.col(col++) = value.mean();
        }

        return zs_voxel;
    }

    ankerl::unordered_dense::map<Key, Value3D> _voxel_map;
    float _resolution; ///< Spatial grid cell resolution in meters per cell (m/cell).
    float _inverse_resolution;
};