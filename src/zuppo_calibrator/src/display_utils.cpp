#include "display_utils.h"
#include "draw_primitives.h"

int showCanvas(Canvas &canvas, const std::string &window_name, int timeout_ms)
{
  cv::imshow(window_name, canvas);
  int key;
  if (timeout_ms < 0)
  {
    key = cv::waitKey(1);
  }
  else
  {
    key = cv::waitKey(timeout_ms);
  }
  if (key == 27) // exit on ESC
    exit(0);
  return key;
}

void renderDebugUI(const std::string &window_name, Canvas &canvas,
                   const GridMap &gmap, const Eigen::Isometry2f &pose,
                   const std::vector<Eigen::Vector2f> &points, const cv::Scalar &point_color)
{
  if (!points.empty())
  {
    drawPoints(canvas, gmap, points, point_color, pose);
  }
  drawCircle(canvas, gmap.worldToGrid(pose.translation()), 2, cv::Scalar(0, 255, 0));
  cv::imshow(window_name, canvas);
}