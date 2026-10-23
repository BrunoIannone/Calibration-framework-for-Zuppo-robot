#pragma once

#include <cstdint>
#include <vector>
#include <Eigen/Core>

/**
 * @brief A generic 2D dense grid container. Stores data in a flat array in row-major order.
 * * @note This class provides spatial lookup, element interpolation, and basic image/grid processing utilities.
 * * @tparam CellType_ The data type stored in each grid cell (e.g., float, uint8_t, Eigen::Vector2f).
 */
template <typename CellType_>
struct Grid_
{
  using CellType = CellType_;                             ///< Grid cell type
  using CellGradientType = Eigen::Matrix<CellType, 2, 1>; ///< Gradient grid cell type
  using GridGradientType = Grid_<CellGradientType>;       ///< Gradient grid type
  using CellTypeVector = std::vector<CellType_>;          ///< Vector containing all cells of CellType_

  /**
   * @brief Constructs a new Grid object
   * * @param rows Initial number of rows. Defaults to 0.
   * @param cols Initial number of columns. Defaults to 0.
   */
  Grid_(size_t rows = 0, size_t cols = 0)
  {
    resize(rows, cols);
  }

  /**
   * @brief Destructor. Automatically clears allocated memory.
   */
  ~Grid_()
  {
    clear();
  }

  // MEMORY

  /**
   * @brief Resizes the grid to new dimensions.
   * * Do nothing if the target dimensions match the current dimensions.
   * * @param r Target number of rows.
   * @param c Target number of columns.
   */
  inline void resize(size_t r, size_t c)
  {
    if (r == _rows && c == _cols)
      return;
    _rows = r;
    _cols = c;
    _values.resize(_rows * _cols);
  }

  /**
   * @brief Clears the grid data and resets dimensions to 0x0.
   *
   */
  inline void clear()
  {
    _values.clear();
    _rows = 0;
    _cols = 0;
  }

  /**
   * @brief Gets the total number of cells in the grid.
   * @return The total number of elements (rows * columns).
   */
  inline size_t size() const { return _rows * _cols; }

  // ELEMENT ACCESS

  /**
   * @brief Read-write access to a grid cell at a specific row and column.
   * * @param r Row index.
   * @param c Column index.
   * @return Reference to the cell data.
   */
  inline CellType_ &at(int r, int c)
  {
    if (r < 0 || static_cast<size_t>(r) >= _rows || c < 0 || static_cast<size_t>(c) >= _cols)
      throw std::out_of_range("Grid_::at() - Index out of bounds");
    return _values[static_cast<size_t>(r) * _cols + static_cast<size_t>(c)];
  }

  /**
   * @brief Read-only access to a grid cell at a specific row and column.
   * * @param r Row index.
   * @param c Column index.
   * @return Const reference to the cell data.
   */
  inline const CellType_ &at(int r, int c) const
  {
    if (r < 0 || static_cast<size_t>(r) >= _rows || c < 0 || static_cast<size_t>(c) >= _cols)
      throw std::out_of_range("Grid_::at() - Index out of bounds");
    return _values[static_cast<size_t>(r) * _cols + static_cast<size_t>(c)];
  }

  /**
   * @brief Checks if the given row and column indices fall within valid grid boundaries.
   * * @param r Row index.
   * @param c Column index.
   * @return true if the indices are within bounds, false otherwise.
   */
  inline bool inside(int r, int c) const
  {
    return r >= 0 && static_cast<size_t>(r) < _rows && c >= 0 && static_cast<size_t>(c) < _cols;
  }

  /**
   * @brief Computes the [row, col] indices given a pointer to a cell.
   * * @param ptr Pointer to a cell within this grid.
   * @return Eigen::Vector2i containing [row, col].
   */
  inline Eigen::Vector2i pos(const CellType_ *ptr) const
  {
    const auto offset = static_cast<std::size_t>(ptr - &_values[0]);
    return Eigen::Vector2i(offset / _cols, offset % _cols);
  }

  /**
   * @brief Computes the [row, col] indices given a pointer to a cell. Used for distance map parent indexing.
   *
   * @param ptr Pointer to a cell within this grid.
   * @return std::pair<int, int> of [row, column] indices.
   */
  inline std::pair<int, int> getCoordinatesFromPointer(const CellType *ptr) const
  {

    const size_t idx = static_cast<size_t>(ptr - &_values[0]);
    return std::make_pair(idx / _cols, idx % _cols);
  }

  // ALGORITHMS

  /**
   * @brief Performs bilinear interpolation
   * * @param r Float row index.
   * @param c Float column index.
   * @return std::pair<CellType, bool> where:
   *
   * - First: The interpolated CellType value (or default-constructed if out of bounds).
   *
   * - Second: A success boolean flag (true if inside safe bounds, false otherwise).
   */
  inline std::pair<CellType, bool> atBilinear(float r, float c) const
  {
    const int ir0 = std::floor(r);
    const int ic0 = std::floor(c);

    // Safeguard to ensure neighbors (ir0 + 1, ic0 + 1) don't overflow grid size
    if (ir0 < 0 || static_cast<size_t>(ir0) > _rows - 2 || ic0 < 0 || static_cast<size_t>(ic0) > _cols - 2)
      return std::make_pair(CellType(), false);

    // Fetch the 4 neighboring grid values
    const CellType &f00 = at(ir0, ic0);
    const CellType &f01 = at(ir0, ic0 + 1);
    const CellType &f10 = at(ir0 + 1, ic0);
    const CellType &f11 = at(ir0 + 1, ic0 + 1);

    // Compute fractional weights
    const float dr = r - ir0;
    const float dc = c - ic0;

    // Standard bilinear blending formula
    CellType interpolated = f00 * (1.f - dr) * (1.f - dc) +
                            f01 * (1.f - dr) * dc +
                            f10 * dr * (1.f - dc) +
                            f11 * dr * dc;

    return std::make_pair(interpolated, true);
  }

  /**
   * @brief Computes the spatial gradient of the grid using central differences.
   * * @return GridGradientType A new grid of identical dimensions, where each cell contains
   * an Eigen::Vector2 (representing [d_row, d_col]).
   */
  GridGradientType gradient() const
  {
    GridGradientType grad(_rows, _cols);
    std::fill(grad._values.begin(), grad._values.end(), CellGradientType(0, 0));

    for (size_t r = 1; r < _rows - 1; ++r)
    {
      for (size_t c = 1; c < _cols - 1; ++c)
      {
        grad.at(r, c) = CellGradientType(at(r + 1, c) - at(r - 1, c),
                                         at(r, c + 1) - at(r, c - 1));
      }
    }
    return grad;
  }

  /**
   * @brief Casts the grid components data type to a new cell type.
   * * @tparam DestCellType The desired data type of the destination grid's cells.
   * @return Grid_<DestCellType> A new grid whose entries are the same but of DestCellType.
   */
  template <typename DestCellType>
  Grid_<DestCellType> cast() const
  {
    Grid_<DestCellType> dest(_rows, _cols);
    for (size_t i = 0; i < _values.size(); ++i)
    {
      dest._values[i] = static_cast<DestCellType>(_values[i]);
    }
    return dest;
  }

  // MEMBER VARIABLES

  size_t _rows = 0;       ///< Number of rows
  size_t _cols = 0;       ///< Number of cols
  CellTypeVector _values; ///< Vector containing all cells of CellType_
};