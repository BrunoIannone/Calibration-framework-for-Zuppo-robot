#pragma once

#include <limits>
#include <vector>

#include "grid_map.h"
#include "grid_mapping.h"

/**
 * @brief Represents a single cell within the distance map.
 * Stores a pointer to its closest obstacle cell (parent).
 */
struct DMapCell
{
  DMapCell *parent = nullptr; ///< Pointer to the parent cell (closest obstacle).
};

/**
 * @brief A grid map containing distances from obstacles information
 *
 * Distance values are stored as uint32_t values.
 *
 * This class inherits structural grid properties from Grid_ and spatial coordinate
 * transformations from GridMapping.
 *
 * The class is designed to be used for Localization or Path Planning.
 */
struct DistanceMap : public Grid_<DMapCell>, public GridMapping
{
  /**
   * @brief Constructs an empty DistanceMap with default dimensions and settings.
   */
  DistanceMap() : Grid_(), GridMapping() {}

  /**
   * @brief Constructs a new DistanceMap object.
   * * @param rows Number of rows.
   * @param cols Number of columns.
   * @param resolution [meters/cell].
   * @param map_origin Map origin position in the world frame [meters].
   */
  DistanceMap(size_t rows, size_t cols, float resolution, Eigen::Vector2f map_origin)
      : Grid_<DMapCell>(rows, cols), GridMapping(resolution, map_origin) {}

  /**
   * @brief Gets the maximum allowed squared distance value.
   * @return The maximum squared distance value.
   */
  inline uint32_t d2_max() const { return _d2_max; }

  /**
   * @brief Initializes the grid using a GridMap.
   * After gathering all the obstacles, the function computes the distance map.
   *
   * @param map The input grid map containing occupancy data.
   * @param d2_max Maximum distance squared allowed in the distance map computation.
   */
  void loadFromGridMap(
      const GridMap &map,
      uint32_t d2_max = std::numeric_limits<uint32_t>::max());

  /**
   * @brief Computes the distance map given a set of obstacles.
   *
   * @param obstacles A vector of (row, col) coordinates representing obstacle locations.
   * @param d2_max Maximum distance squared allowed in the distance map computation.
   * @return The calculated maximum squared distance or return max value.
   */
  uint32_t compute(
      const std::vector<std::pair<uint32_t, uint32_t>> &obstacles,
      uint32_t d2_max = std::numeric_limits<uint32_t>::max());

  /**
   * @brief Computes the squared distance between two cells in grid space.
   * The distance is computed as the squared Euclidean distance between the
   * cells' positions.
   *
   * @param c1 The first cell.
   * @param c2 The second cell.
   * @return The squared Euclidean distance between c1 and c2.
   */
  inline uint32_t squaredDistance(const DMapCell &c1,
                                  const DMapCell &c2) const
  {
    const auto p1 = getCoordinatesFromPointer(&c1);
    const auto p2 = getCoordinatesFromPointer(&c2);
    return static_cast<uint32_t>((p1.first - p2.first) * (p1.first - p2.first) +
                                 (p1.second - p2.second) * (p1.second - p2.second));
  }

  /**
   * @brief Computes the squared distance from a cell to its assigned parent (closest obstacle).
   * * @param c Pointer to the cell to query.
   * @return The squared distance to the parent, or max int if the cell has no parent.
   */
  inline int parentDistance(const DMapCell *c) const
  {
    if (!c->parent)
      return std::numeric_limits<int>::max();
    Eigen::Vector2i parent_pos = pos(c->parent);
    Eigen::Vector2i c_pos = pos(c);
    return (parent_pos - c_pos).squaredNorm();
  }

  /**
   * @brief Extracts the squared distances from the DistanceMap and stores them in the
   * target grid.
   *
   * @tparam OutputCellType_ Output grid type, expected [i.e. float, double, uint,...]
   * @param target Output grid
   * @param d2_max Maximum distance squared allowed in the distance map
   */
  template <typename OutputCellType_>
  inline void extractDistancesSquared(Grid_<OutputCellType_> &target,
                                      uint32_t d2_max) const
  {
    target.resize(_rows, _cols);
    for (size_t i = 0; i < _values.size(); ++i)
    {
      const auto &cell = _values[i];
      uint32_t d2 = d2_max;
      if (cell.parent)
      {
        d2 = squaredDistance(cell, *cell.parent);
      }
      target._values[i] = static_cast<OutputCellType_>(d2);
    }
  }

  /**
   * @brief Generates a grid containing the non-squared distance values.
   * * @param max_distance Cap value for the distance calculation.
   * @return A grid of floats containing the distance map values.
   */
  Grid_<float> distances(float max_distance = std::numeric_limits<float>::max()) const;

  // MEMBER VARIABLES
  uint32_t _d2_max;                                      ///< The maximum allowed squared distance in the map.
  std::vector<std::pair<uint32_t, uint32_t>> _obstacles; ///< Vector containing all the obstacles coordinates
};