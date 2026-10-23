#include "hlidar_calibrator.h"

HLidarCalibrator::HLidarCalibrator(
    Zuppo &zuppo,
    const uint8_t lidar_idx,
    const float max_error,
    const float kernel_threshold,
    const float damping)
    : Calibrator<6>(zuppo, max_error, kernel_threshold),
      _kl(_zuppo._kl),
      _kr(_zuppo._kr),
      _wb(_zuppo._wb),
      _r_X_l(iso3dTo2d(_zuppo._lidar_vec[lidar_idx]._r_X_l)),
      _lidar_idx(lidar_idx),
      _damping(damping)
{
}

std::vector<float> HLidarCalibrator::calibrate(const std::vector<HCalibrationStep> &steps, const size_t max_iterations)
{
  std::vector<float> chi_history;
  chi_history.reserve(max_iterations);

  for (size_t iter = 0; iter < max_iterations; ++iter)
  {
    reset();

    StateVector x = getState();

    for (const auto &step : steps)
    {
      addMeasurement(x, step);
    }

    update(x);

    chi_history.push_back(_chi2);
  }
  _zuppo._lidar_vec[_lidar_idx]._r_X_l = iso2dTo3d(_r_X_l);

  return chi_history;
}

void HLidarCalibrator::addMeasurement(const StateVector &x, const HCalibrationStep &step)
{
  const Eigen::Isometry2f &T_a = step.T_a;
  const Eigen::Isometry2f &T_b = step.T_b;

  const Eigen::Isometry2f l_Z_lp = v2t(boxminus(T_b, T_a));
  const Eigen::Isometry2f l_X_lp = predFunction(x, step.ticks);

  Eigen::Vector3f e = boxminus(l_X_lp, l_Z_lp);

  const float this_chi = e.squaredNorm();
  float kscale = 1;
  if (this_chi > _max_error)
  {
    ++_num_outliers;
    return;
  }
  else if (this_chi > _kernel_threshold)
  {
    kscale = std::sqrt(_kernel_threshold / this_chi);
    ++_num_outliers;
  }
  else
  {
    ++_num_inliers;
  }
  _chi2 += this_chi * kscale;

  // Numerical Jacobian
  Eigen::Matrix<float, 3, 6> J;
  const float epsilon = 1e-6f;
  for (uint8_t k = 0; k < 6; ++k)
  {
    StateVector dx_plus = StateVector::Zero();
    StateVector dx_minus = StateVector::Zero();

    dx_plus(k) = epsilon;
    dx_minus(k) = -epsilon;

    const StateVector x_plus = boxplus(x, dx_plus);
    const StateVector x_minus = boxplus(x, dx_minus);

    const Eigen::Vector3f e_plus = boxminus(predFunction(x_plus, step.ticks), l_Z_lp);
    const Eigen::Vector3f e_minus = boxminus(predFunction(x_minus, step.ticks), l_Z_lp);

    const Eigen::Vector3f diff = e_plus - e_minus;
    J.col(k) = diff / (2.0f * epsilon);
  }
  H.noalias() += (J.transpose() * J) * kscale;
  b.noalias() -= (J.transpose() * e) * kscale;
}

void HLidarCalibrator::update(StateVector &x)
{
  H += Eigen::Matrix<float, 6, 6>::Identity() * _damping;
  dx = H.ldlt().solve(b);

  x = boxplus(x, dx);
  _kl = x(0);
  _kr = x(1);
  _wb = x(2);
  _r_X_l = v2t<float>(x.tail<3>());
}