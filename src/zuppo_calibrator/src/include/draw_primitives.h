#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "grid_mapping.h"

using Canvas = cv::Mat;

/**
 * @brief Draws a line on a canvas.
 *
 * @param dest Destination canvas.
 * @param p0 Start point.
 * @param p1 End point.
 * @param color Color of the line.
 */
inline void drawLine(Canvas &dest, const Eigen::Vector2i &p0,
                     const Eigen::Vector2i &p1, cv::Scalar color, int thickness = 1)
{
    cv::line(dest, cv::Point(p0.x(), p0.y()), cv::Point(p1.x(), p1.y()),
             color, thickness);
}

/**
 * @brief Draws a circle on a canvas.
 *
 * @param dest Destination canvas.
 * @param center Center of the circle.
 * @param radius Radius of the circle.
 * @param color Color of the circle.
 */
inline void drawCircle(Canvas &dest, const Eigen::Vector2i &center, int radius,
                       cv::Scalar color, int thickness = 2)
{
    cv::circle(dest, cv::Point(center.x(), center.y()), radius,
               color, thickness);
}

/**
 * @brief Draw 2D points onto a canvas image
 * @param dest Destination canvas.
 * @param mapping Grid mapping object
 * @param X Transformation isometry that brings the points in the world frame. Default is Identity
 * @param points Array of 2D points in local coordinate space
 * @param intensity BGR color value used for drawing points
 * @note Points are assumed to be in any frame, to be correctly drawn you need to pass X such that w_p = X * p
 */
void drawPoints(Canvas &dest,
                const GridMapping &mapping,
                const std::vector<Eigen::Vector2f> &points,
                const cv::Scalar &intensity,
                const Eigen::Isometry2f &X = Eigen::Isometry2f::Identity());
