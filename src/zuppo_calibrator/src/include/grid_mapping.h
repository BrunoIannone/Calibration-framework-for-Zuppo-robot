#pragma once

#include <Eigen/Core>

/**
 * @brief Manages 2D coordinate transformations between map space and world space to work with openCV.
 *
 * OpenCV grid origin is defined in the top left corner. Y axis points downwards, and X axis points to the right.
 * The Map origin with respect to the world frame might varies, but we consider Y axis pointing upwards, and X axis pointing to the right.
 * The world frame origin is defined in the center of the map. We consider Y axis pointing upwards, and X axis pointing to the right.
 *
 * Therefore, this struct will handle the conversion to OpenCV grid coordinates, and vice versa.
 */
struct GridMapping
{

  /**
   * @brief Constructs an uninitialized GridMapping object.
   */
  GridMapping()
      : _resolution(0.0f), _inv_resolution(0.0f), _map_origin(Eigen::Vector2f::Zero()) {}

  /**
   * @brief Constructs a GridMapping object.
   * @param resolution [meters / cell].
   * @param map_origin Map origin position in the world frame [meters].
   */
  GridMapping(float resolution, Eigen::Vector2f map_origin)
      : _resolution(resolution), _inv_resolution(1.f / _resolution), _map_origin(map_origin)
  {
  }

  // COORDINATE TRANSFORMATIONS

  /**
   * @brief Converts a pixel coordinate to a world position.
   * * @param pixel 2D coordinate inside the grid space.
   * @return Eigen::Vector2f 2D position vector [x, y] in world space (meters).
   */
  inline Eigen::Vector2f gridToWorld(const Eigen::Vector2i &pixel) const
  {
    Eigen::Vector2f dest;
    dest.x() = (static_cast<float>(pixel.x()) * _resolution) + _map_origin.x();
    dest.y() = (-static_cast<float>(pixel.y()) * _resolution) - _map_origin.y();
    return dest;
  }

  // COORDINATE TRANSFORMATIONS (World to Grid)

  /**
   * @brief Converts a world point into discrete grid cell indices.
   * * @param point 2D position vector [x, y] in world space (meters).
   * @return Eigen::Vector2i 2D cell index vector [col, row].
   */
  inline Eigen::Vector2i worldToGrid(const Eigen::Vector2f &point) const
  {
    Eigen::Vector2i dest;
    dest.x() = static_cast<int>(std::floor((point.x() - _map_origin.x()) * _inv_resolution));
    dest.y() = static_cast<int>(std::floor((-point.y() - _map_origin.y()) * _inv_resolution));

    return dest;
  }

  /**
   * @brief Converts a world point into float grid coordinates. This is useful for interpolation.
   * * @param point 2D position vector [x, y] in world space (meters).
   * @return Eigen::Vector2f 2D cell index vector [col, row].
   */

  inline Eigen::Vector2f worldToGridF(const Eigen::Vector2f &point) const
  {
    Eigen::Vector2f dest;
    dest.x() = (point.x() - _map_origin.x()) * _inv_resolution;
    dest.y() = (-point.y() - _map_origin.y()) * _inv_resolution;
    return dest;
  }

  // MEMBER VARIABLES

  float _resolution;     ///< [meters/cell].
  float _inv_resolution; ///< inverse resolution [cell / meter].

  Eigen::Vector2f _map_origin; ///< Map origin position in the world frame [meters].
};