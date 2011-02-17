#include <vw/orbital_refinement/TrajectoryErrorEstimator.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>

using vw::Vector3;

TrajectoryErrorEstimator::TrajectoryErrorEstimator(
    const std::list<OrbitalReading>& observations) 
        : _observations(observations)
{
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

    // Calculate error and gradient for the first point
  double error_squared =
      calculatePositionError(0, p0, v0, *observation_iter, calculate_gradient);
  
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

    // Save the gradients for the non-time values, if requested
  if (calculate_gradient)
  {
      // Gradient for GM
    double new_error;
    if (GM == 0)
      _gradient.GM = 0;
    else
    {
      new_error = calculateError(GM*1.001, p0, v0, t, false);
      _gradient.GM = (new_error - error_squared)/(.001*GM);
    }

      // Gradient for each element of p0
    for (int i = 0; i < 3; ++i)
    {
      if (p0[i] != 0)
      {
        double save = p0[i];
        p0[i] *= 1.001;
        new_error = calculateError(GM, p0, v0, t, false);
        p0[i] = save;
        _gradient.p0[i] = (new_error - error_squared)/(.001*p0[i]);
      }
      else
        _gradient.p0[i] = 0;
    }
    
      // Gradient for each element of v0
    for (int i = 0; i < 3; ++i)
    {
      if (v0[i] != 0)
      {
        double save = v0[i];
        v0[i] *= 1.001;
        new_error = calculateError(GM, p0, v0, t, false);
        v0[i] = save;
        _gradient.v0[i] = (new_error - error_squared)/(.001*v0[i]);
      }
      else
        _gradient.v0[i] = 0;
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
  Vector3 this_err = calculated_position - observation.mCoord;
  
  if (calculate_gradient)
    _gradient.t[index] = 2*dot_prod(this_err,calculated_velocity);
  
  return dot_prod(this_err,this_err);
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
