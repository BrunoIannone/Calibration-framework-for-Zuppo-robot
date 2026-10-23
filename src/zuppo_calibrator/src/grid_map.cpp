#include "grid_map.h"
#include "params.h"

void GridMap::loadFromImage(const std::string &filename, float res, const Eigen::Vector2f &map_origin)
{

  std::cout << YELLOW << "\nLoading map image:" << RESET << "[" << filename << "]" << std::endl;

  const cv::Mat m = cv::imread(filename, cv::IMREAD_UNCHANGED);
  if (m.rows == 0 || m.channels() != 4)
  {
    throw std::runtime_error(
        "unable to load image or image does not have 4 channels");
  }
  cv::Mat loaded_image;
  cv::cvtColor(m, loaded_image, cv::COLOR_BGRA2GRAY);
  Grid_::resize(static_cast<uint32_t>(loaded_image.rows), static_cast<uint32_t>(loaded_image.cols));

  _resolution = res;
  _inv_resolution = 1.f / _resolution;
  GridMapping::_map_origin = map_origin;

  for (int r = 0; r < m.rows; ++r)
  {
    for (int c = 0; c < m.cols; ++c)
    {
      // Read the grayscale value from the loaded image
      const uint8_t &grayscale_value = loaded_image.at<uint8_t>(r, c);
      // Read the transparency (alpha channel) from the original image
      const uint8_t &alpha_value = m.at<cv::Vec4b>(r, c)[3];
      // Set the grid cell value based on the alpha and grayscale values
      if (alpha_value < 127)
      {
        Grid_::at(r, c) = UNKNOWN;
      }
      else if (grayscale_value < 147)
      {
        Grid_::at(r, c) = OCCUPIED;
      }
      else
      {
        Grid_::at(r, c) = FREE;
      }
    }
  }
  std::cerr << GREEN << "Map loading complete: \n"
            << RESET << "  -Resolution: " << _resolution << "\n"
            << "  -Inv resolution: " << _inv_resolution << "\n"
                                                           "  -Map origin: "
            << "(" << _map_origin.x() << "," << _map_origin.y() << ")\n"
            << RESET << std::endl;
}

Canvas drawGridMap(Canvas &dest, const Grid_<uint8_t> &src, const uint8_t free,
                   const uint8_t occupied, const uint8_t unknown)
{
  dest = cv::Mat(src._rows, src._cols, CV_8UC1);
  for (size_t i = 0; i < src._rows; ++i)
  {
    for (size_t j = 0; j < src._cols; ++j)
    {
      const uint8_t &value = src._values[i * src._cols + j];
      if (value == free)
      {
        dest.at<uint8_t>(i, j) = 255;
      }
      else if (value == occupied)
      {
        dest.at<uint8_t>(i, j) = 0;
      }
      else if (value == unknown)
      {
        dest.at<uint8_t>(i, j) = 200;
      }
      else
      {
        dest.at<uint8_t>(i, j) = value; // Preserve other values
      }
    }
  }
  return dest;
}

Canvas GridMap::draw(Canvas &dest) const
{
  return drawGridMap(dest, *this, FREE, OCCUPIED, UNKNOWN);
}
