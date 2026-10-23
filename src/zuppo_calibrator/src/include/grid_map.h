#pragma once

#include "grid.h"
#include "grid_mapping.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using Canvas = cv::Mat;

/**
 * @brief A 2D grid map containing occupancy information.
 * * Occupancy values are represented as uint8_t values (FREE, OCCUPIED, UNKNOWN).
 * This class inherits structural grid properties from Grid_ and spatial coordinate
 * transformations from GridMapping.
 */
struct GridMap : public Grid_<uint8_t>, public GridMapping
{
  static const uint8_t FREE = 0;       ///< Cell is safe and traversable.
  static const uint8_t OCCUPIED = 100; ///< Cell contains an obstacle.
  static const uint8_t UNKNOWN = 255;  ///< Cell has not been observed yet.

  using Grid_::at;
  using Grid_::inside;

  /**
   * @brief Constructs an empty GridMap with default dimensions and settings.
   */
  GridMap() : Grid_(), GridMapping() {}

  /**
   * @brief Constructs a new GridMap object.
   * * @param rows Number of rows.
   * @param cols Number of columns.
   * @param resolution [meters/cell].
   * @param map_origin Map origin position in the world frame [meters].
   */
  GridMap(size_t rows, size_t cols, float resolution, Eigen::Vector2f map_origin)
      : Grid_<uint8_t>(rows, cols), GridMapping(resolution, map_origin) {}

  // MEMORY

  /**
   * @brief Resizes the grid to new dimensions. Update resolution.
   * * Do nothing if the target dimensions match the current dimensions.
   * * @param r Target number of rows.
   * @param c Target number of columns.
   * @param res Target resolution [meters/cell].
   */
  void resize(size_t r, size_t c, float res)
  {
    Grid_::resize(r, c);
    _resolution = res;
    _inv_resolution = 1.f / _resolution;
  }

  // ELEMENT ACCESS

  /**
   * @brief Read-write access to a grid cell using world coordinates.
   * * @param point 2D spatial position vector [x, y] in world coordinates (meters).
   * @return Reference to the cell occupancy value.
   */
  inline uint8_t &at(Eigen::Vector2f point)
  {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::at(idx.y(), idx.x());
  }

  /**
   * @brief Read-only access to a grid cell using world coordinates.
   * * @param point 2D spatial position vector [x, y] in world coordinates (meters).
   * @return Const reference to the cell occupancy value.
   */
  inline const uint8_t &at(Eigen::Vector2f point) const
  {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::at(idx.y(), idx.x());
  }

  /**
   * @brief Checks if a continuous world point falls inside the map boundaries.
   * * @param point 2D spatial position vector [x, y] in world coordinates (meters).
   * @return true if the mapped point is within grid boundaries, false otherwise.
   */
  inline bool inside(Eigen::Vector2f point) const
  {
    Eigen::Vector2i idx = worldToGrid(point);
    return Grid_::inside(idx.y(), idx.x());
  }

  /**
   * @brief Explicitly checks if a cell is marked as blocked/obstacle.
   * * @param row Discrete grid row index.
   * @param col Discrete grid column index.
   * @return true if the target cell state matches the OCCUPIED threshold.
   */
  inline bool isOccupied(size_t row, size_t col) const
  {
    return Grid_::at(row, col) == OCCUPIED;
  }

  // ALGORITHMS

  /**
   * @brief Populates the GridMap data by importing an external image file.
   * * @param filename Map image path.
   * @param resolution Map resolution [meters/cell].
   * @param map_origin The origin of the map with respect to the world frame (in meters)
   */
  void loadFromImage(const std::string &filename, float resolution, const Eigen::Vector2f &map_origin);

  // VISUALIZATION

  /**
   * @brief Draws a GridMap on a canvas. It can be shown with OpenCV.
   *
   * @param dest Destination canvas.
   * @return Canvas
   */
  Canvas draw(Canvas &dest) const;
};