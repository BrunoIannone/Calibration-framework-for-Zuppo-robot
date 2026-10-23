#include "distance_map.h"

#include <iostream>
#include <queue>

void DistanceMap::loadFromGridMap(const GridMap &map,
                                  uint32_t d2_max)
{
  _rows = map._rows;
  _cols = map._cols;
  _values.resize(_rows * _cols);
  _map_origin =
      map._map_origin;

  _resolution = map._resolution;
  _inv_resolution = map._inv_resolution;
  _d2_max = d2_max;

  for (size_t r = 0; r < _rows; ++r)
  {
    for (size_t c = 0; c < _cols; ++c)
    {
      const size_t i = _cols * r + c; //<- choose this to flip the input image
      // unsigned int i = _cols * (_rows - r - 1) + c; //<- choose this to flip the input image
      uint8_t value = map._values[i];

      Grid_::at(r, c).parent = nullptr;
      if (value == GridMap::OCCUPIED)
      {
        _obstacles.push_back(std::make_pair(r, c));
      }
    }
  }

  compute(_obstacles, d2_max);
}

uint32_t DistanceMap::compute(
    const std::vector<std::pair<uint32_t, uint32_t>> &obstacles,
    uint32_t d2_max)
{
  _d2_max = d2_max;
  std::queue<DMapCell *> frontier;

  // Initialize the frontier by pushing the obstacles into it
  for (const auto &obs : obstacles)
  {
    const auto row = obs.first, col = obs.second;
    // Check if the obstacle is inside the grid
    if (!Grid_::inside(static_cast<int>(row), static_cast<int>(col)))
    {
      continue;
    }
    // Initialize the DMapCell by setting the parent to its own cell
    auto &cell = at(static_cast<int>(row), static_cast<int>(col));
    at(static_cast<int>(row), static_cast<int>(col)).parent = &cell;
    frontier.push(&cell);
  }

  uint32_t steps = 0;
  while (!frontier.empty())
  {
    ++steps;

    // Fetch one node from the frontier
    auto cell = frontier.front();
    frontier.pop();

    // Fetch the cell's position and its parent
    const auto cell_position = getCoordinatesFromPointer(cell);
    const auto cell_row = cell_position.first;
    const auto cell_col = cell_position.second;
    const auto cell_parent = cell->parent;

    // Visit all neighbors
    for (int8_t dr = -1; dr <= 1; ++dr)
    {
      for (int8_t dc = -1; dc <= 1; ++dc)
      {
        // Skip the current cell
        if (dr == 0 and dc == 0)
        {
          continue;
        }

        // Compute the neighbor's position
        int32_t neighbor_row = cell_row + dr;
        int32_t neighbor_col = cell_col + dc;

        // Check if the neighbor is inside the grid
        if (!Grid_::inside(neighbor_row, neighbor_col))
        {
          continue;
        }

        // Fetch the neighbor's cell
        auto &neighbor = at(neighbor_row, neighbor_col);

        // cell's parent
        const auto d2_neighbor = squaredDistance(neighbor, *cell_parent);
        if (d2_neighbor > d2_max)
        {
          continue;
        }
        // If the neighbor has no parent or the distance to the parent is
        // smaller than the current distance, update the parent and push the
        // neighbor to the frontier
        if (!neighbor.parent or
            d2_neighbor < squaredDistance(*neighbor.parent, neighbor))
        {
          neighbor.parent = cell_parent;
          frontier.push(&neighbor);
        }
      }
    }
  }
  return steps;
}

Grid_<float> DistanceMap::distances(float max_distance) const
{
  Grid_<float> dest(_rows, _cols);
  for (size_t r = 0; r < _rows; ++r)
  {
    for (size_t c = 0; c < _cols; ++c)
    {
      const auto &cell = at(r, c);
      dest.at(r, c) = std::min(max_distance, sqrtf(parentDistance(&cell)));
    }
  }
  return dest;
}