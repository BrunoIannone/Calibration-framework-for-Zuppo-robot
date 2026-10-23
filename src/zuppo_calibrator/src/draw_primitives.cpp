#include "draw_primitives.h"

void drawPoints(Canvas &dest,
                const GridMapping &mapping,
                const std::vector<Eigen::Vector2f> &points,
                const cv::Scalar &intensity,
                const Eigen::Isometry2f &X)
{

    for (const auto &point : points)
    {
        const auto world_points = X * point;

        const auto pg =
            mapping.worldToGrid(
                world_points);

        if (pg.x() >= 0 && pg.x() < dest.cols && pg.y() >= 0 && pg.y() < dest.rows)
        {

            cv::circle(dest,
                       cv::Point(pg.x(), pg.y()),
                       2,
                       intensity);
        }
    }
}
