#pragma once

#include <Eigen/Core>

#include "zuppo.h"

/**
 * @brief Base class for sensor calibration using Gauss-Newton / Levenberg-Marquardt optimization.
 * @tparam StateDim The dimension of the state vector.
 */
template <int StateDim>

class Calibrator
{
public:
    using StateVector = Eigen::Matrix<float, StateDim, 1>;    ///< Type alias for the optimization state vector (N x 1).
    using MatrixH = Eigen::Matrix<float, StateDim, StateDim>; ///< Type alias for the Hessian approximation matrix (N x N).

    /**
     * @brief Constructs a Calibrator instance.
     *
     * @param zuppo Reference to Zuppo.
     * @param max_error Maximum error threshold for a point to be an inlier
     * @param kernel_threshold Threshold for downweighting points with errors that exceed acceptable limits yet remain below max_error
     */
    Calibrator(Zuppo &zuppo, float max_error = 0.1f, float kernel_threshold = 0.001f)
        : _max_error(max_error),
          _kernel_threshold(kernel_threshold),
          _zuppo(zuppo)
    {
        reset();
    }

    /**
     * @brief Virtual destructor.
     */
    virtual ~Calibrator() = default;

    /**
     * @brief Resets the linear system accumulators and optimization metrics to zero.
     */
    virtual inline void reset()
    {
        H.setZero();
        b.setZero();
        dx.setZero();
        _chi2 = 0.0f;
        _num_inliers = 0;
        _num_outliers = 0;
    }

protected:
    float _chi2;
    float _max_error;
    float _kernel_threshold;

    uint32_t _num_inliers;
    uint32_t _num_outliers;

    Zuppo &_zuppo;

    MatrixH H;
    StateVector b;
    StateVector dx;
};