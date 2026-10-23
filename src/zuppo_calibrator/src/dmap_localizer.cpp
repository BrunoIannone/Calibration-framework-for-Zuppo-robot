#include "dmap_localizer.h"
#include <Eigen/Cholesky>

DMapLocalizer::DMapLocalizer(const DistanceMap &dmap, float max_range,
                             bool use_bilinear, float point_damping, float kernel_threshold, float max_error) : DistanceMap(dmap._rows, dmap._cols, dmap._resolution, dmap._map_origin),
                                                                                                                _point_damping(point_damping),
                                                                                                                _kernel_threshold(kernel_threshold),
                                                                                                                _use_bilinear(use_bilinear),
                                                                                                                _max_error(max_error)

{

  const float max_range_in_pixel = max_range * _inv_resolution;

  _distances = dmap.distances(max_range_in_pixel);

  _distance_gradients = _distances.gradient();

  for (auto &d : _distances._values)
  {
    d *= _resolution;
  }
  X = Eigen::Isometry2f::Identity();

  reset();
}

void DMapLocalizer::reset()
{
  H.setZero();
  b.setZero();
  dx.setZero();
  _chi2 = 0;
  _num_inliers = 0;
  _num_outliers = 0;
  _total_damping = 0;
}

bool DMapLocalizer::fetchDistanceAndGradients(float &distance,
                                              Eigen::Vector2f &gradients,
                                              const Eigen::Vector2f &pg) const
{
  if (_use_bilinear)
  {
    const auto d_pair = _distances.atBilinear(pg.y(), pg.x());
    if (!d_pair.second)
      return false;
    distance = d_pair.first;
    gradients = _distance_gradients.atBilinear(pg.y(), pg.x()).first;
    return true;
  }
  else
  {
    const Eigen::Vector2i pi = pg.cast<int>();
    if (!_distances.inside(pi.y(), pi.x()))
      return false;
    distance = _distances.at(pi.y(), pi.x());
    gradients = _distance_gradients.at(pi.y(), pi.x());
    return true;
  }
}

void DMapLocalizer::addMeasurement(const Eigen::Vector2f &l_z)
{
  // 1. map the measurement in the world
  const Eigen::Vector2f p = X * l_z;

  // 2. convert the world point in grid coordinates
  const Eigen::Vector2f pg = worldToGridF(p);

  //  3. fetch the distance value and the gradients
  //     according to the choosen policy
  float e;
  Eigen::Vector2f grad;
  if (!fetchDistanceAndGradients(e, grad, pg) || std::abs(e) > _max_error)
  {
    ++_num_outliers;
    return;
  }
  // 4. fetch the distance (in meters)
  _total_damping += _point_damping;
  const float this_chi = e * e;
  float kscale = 1;
  if (this_chi > _kernel_threshold)
  {
    kscale = sqrt(_kernel_threshold / this_chi);
    ++_num_outliers;
  }
  else
  {
    ++_num_inliers;
  }
  _chi2 += this_chi * kscale;

  // 5. compute the linearization (gauss newton)
  // Notice that the Jacobian is not the normal ICP ones. There is a swap due to the row/cols convention.

  const Eigen::Vector3f J(grad.y(), -grad.x(),
                          -(grad.y() * p.y() + grad.x() * p.x()));
  H.selfadjointView<Eigen::Upper>().rankUpdate(J, kscale);
  b.noalias() -= J * (e * kscale);
}

void DMapLocalizer::update()
{

  H += Eigen::Matrix3f::Identity() * _total_damping;
  dx = H.ldlt().solve(b);
  Eigen::Isometry2f dX; // Manually build the v2t(delta_x): (is the boxplus)
  dX.setIdentity();
  dX.linear() = Eigen::Rotation2Df(dx(2)).toRotationMatrix();
  dX.translation() = dx.head<2>();
  X = dX * X;
}

void DMapLocalizer::localize(const std::vector<Eigen::Vector2f> &l_z, const int max_iterations)
{

  for (int iteration = 0; iteration < max_iterations; ++iteration)
  {
    reset();

    for (const auto &point : l_z)
    {
      addMeasurement(point);
    }
    update();
  }
}