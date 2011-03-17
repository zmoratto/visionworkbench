#include <vw/orbital_refinement/TrajectoryErrorEstimator.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <cassert>

using namespace vw;

TrajectoryErrorEstimator::TrajectoryErrorEstimator(
    const std::list<OrbitalReading>& observations)
        : _default_weights(observations.size(), 1),
          _observations(observations),
          _weights(_default_weights)
{
  _gradient.t.resize(observations.size());
}

TrajectoryErrorEstimator::TrajectoryErrorEstimator(
    const std::list<OrbitalReading>& observations,
    const std::vector<double>& weights) 
        : _observations(observations),
          _weights(weights)
{
  assert(_weights.size() >= observations.size());
  _gradient.t.resize(observations.size());
}

TrajectoryErrorEstimator::result_type
TrajectoryErrorEstimator::operator()(const domain_type& x) const
{
    // This isn't actually a const function because it caches the
    // gradient information.
    // Do a const cast.
  return const_cast<TrajectoryErrorEstimator*>(this)->
      calculateError(x.GM, x.p0, x.v0, x.timestamps, true);
}

inline double TrajectoryErrorEstimator::centralDifferenceGradient(
    double& to_tweak, double epsilon,
    double& GM, Vector3& p0, Vector3& v0,
    const std::vector<OrbitalReading::timestamp_t>& t, double old_error)
{
  double save = to_tweak;
  double forward_x, reverse_x;
  
    // First do the forward difference
  if (fabs(save) < 1.0)
    forward_x = save + epsilon;
  else
    forward_x = save * 1.001;
  to_tweak = forward_x;
  double error = calculateError(GM, p0, v0, t, false);

    // Now do reverse difference
  if (fabs(save) < 1.0)
    reverse_x = save - epsilon;
  else
    reverse_x = save * 0.999;
  to_tweak = reverse_x;
  error -= calculateError(GM, p0, v0, t, false);
  
  to_tweak = save;
  
  return error/(forward_x - reverse_x);
}

TrajectoryErrorEstimator::result_type
TrajectoryErrorEstimator::calculateError(
    double GM, Vector3 p0, Vector3 v0,
    const std::vector<OrbitalReading::timestamp_t>& t, bool calculate_gradient)
{
    // Prepare an appropriate trajectory calculator
  GravityAccelerationFunctor gravity(GM);
  TrajectoryCalculator traj_calc(gravity);

    // There needs to be one observation per timestamp
  assert(t.size() == _observations.size());

    // Since our observations are in a list, it isn't accessible via index.
    // Instead, iterate through the list in sync with the time values.
    // Get the first reading.
  std::list<OrbitalReading>::const_iterator observation_iter =
      _observations.begin();

    // Calculate error and gradient for the first point.
    // Note that gradient is zero for the first t because we're
    // holding t0 constant and adjusting all other t values relative to t0.
  double error_squared =
      calculatePositionError(0, p0, v0, *observation_iter, false);
  _gradient.t[0] = 0;
  
    // Prepare to calculate the rest of the errors and gradients.
  Vector3 prev_position = p0;
  Vector3 prev_velocity = v0;
  Vector3 next_position;
  Vector3 next_velocity;

    // Calculate error and gradient for all remaining points.
  for (std::size_t i = 1; i < t.size(); ++i)
  {
      // Get the next position in the orbit
    traj_calc.calculateNextPoint(prev_position, prev_velocity,
                                 t[i] - t[i-1],
                                 next_position, next_velocity);

      // Move to the next observation
    ++observation_iter;

      // Find the error and, if requested, gradient
    error_squared +=
        calculatePositionError(i, next_position, next_velocity,
                               *observation_iter, calculate_gradient);

      // Get ready for next point
    prev_position = next_position;
    prev_velocity = next_velocity;
  }

  // Calculate the gradients for the non-time values, if requested
  if (calculate_gradient)
  {
    // Gradient for GM
    _gradient.GM=centralDifferenceGradient(GM, .00001, GM, p0, v0, t, error_squared) / 1e18;
    
      // Gradient for each element of p0 and v0
    for (int i = 0; i < 3; ++i)
    {
      _gradient.p0[i] =
          centralDifferenceGradient(p0[i], .001, GM, p0, v0, t, error_squared);
      _gradient.v0[i] =
          centralDifferenceGradient(v0[i], .001, GM, p0, v0, t, error_squared);
    }
  }

  return error_squared;
}

double TrajectoryErrorEstimator::calculatePositionError(
    std::size_t index,
    const Vector3& calculated_position,
    const Vector3& calculated_velocity,
    const OrbitalReading& observation,
    bool calculate_gradient)
{
  if (_weights[index] == 0)
  {
    if (calculate_gradient)
      _gradient.t[index] = 0;
    return 0;
  }
  
  Vector3 this_err = calculated_position - observation.mCoord;
  
  if (calculate_gradient)
    _gradient.t[index] = 2*_weights[index]*dot_prod(this_err,calculated_velocity);

  return _weights[index] * dot_prod(this_err, this_err);
}

// We just return what we've already calculated.  This behavior relies
// on operator() having been called most recently with the same
// x values requested here.
TrajectoryErrorEstimator::gradient_type
TrajectoryErrorEstimator::gradient( domain_type const& x ) const
{
  return _gradient;
}

unsigned TrajectoryErrorEstimator::dimension() const
{
  return _gradient.t.size() + 7;
}
