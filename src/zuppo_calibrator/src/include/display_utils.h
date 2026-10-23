#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>
#include "grid_map.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

using Canvas = cv::Mat;

/**
 * @brief Converts Grid_ object into an OpenCV Mat (Canvas).
 * * @tparam T The pixel/cell type of the source grid.
 * @param src The source Grid_ object.
 * @param normalize If true, the values will be scaled to spans between 0 and 255.
 * @return Canvas (cv::Mat) An 8-bit single-channel OpenCV matrix representation of the grid.
 */
template <typename T>
Canvas grid2cv(const Grid_<T> &src, bool normalize)
{
  // 1. we convert the grid to float
  Grid_<float> float_grid = src.template cast<float>();

  if (normalize)
  {
    // 2. if normalize is selected, we scan for min and max
    float lower = std::numeric_limits<float>::max();
    float upper = std::numeric_limits<float>::min();
    for (const auto &v : float_grid._values)
    {
      lower = std::min(lower, v);
      upper = std::max(upper, v);
    }
    float range = upper - lower;
    if (range == 0)
    {
      std::fill(float_grid._values.begin(),
                float_grid._values.end(),
                0.);
    }
    else
    {
      float irange = 255. / range;
      for (auto &v : float_grid._values)
        v = (v - lower) * irange;
    }
  }
  Canvas dest(float_grid._rows, float_grid._cols, CV_8UC1);
  for (size_t r = 0; r < float_grid._rows; ++r)
  {
    for (size_t c = 0; c < float_grid._cols; ++c)
    {
      dest.at<uint8_t>(r, c) = float_grid.at(r, c);
    }
  }
  return dest;
}

/**
 * @brief Shows a canvas using OpenCV.
 * @param canvas Canvas to show.
 * @param window_name The name of the window to display.
 * @param timeout_ms Timeout in milliseconds. If negative, waits indefinitely until a key is pressed.
 * @return Key pressed.
 */
int showCanvas(Canvas &canvas, const std::string &window_name, int timeout_ms);

/**
 * @brief Render map canvas with robot pose and point cloud
 * @param window_name Title of the OpenCV display window
 * @param canvas Target image canvas to draw on
 * @param gmap Grid map
 * @param pose Current 2D robot pose transformation in the world
 * @param points Array of 2D points to render
 * @param point_color BGR color used for plotting points (defaults to red)
 */
void renderDebugUI(const std::string &window_name, Canvas &canvas,
                   const GridMap &gmap, const Eigen::Isometry2f &pose,
                   const std::vector<Eigen::Vector2f> &points, const cv::Scalar &point_color = cv::Scalar(0, 0, 255));