#include <vw/ORBA/ORBAErrorEstimator.hpp>
#include <vw/Camera/CameraModel.h>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>

#include <boost/foreach.hpp>
#include <cmath>
#include <numeric>

using namespace vw;

// Anonymous namespace for local utility functions
namespace 
{
      // Calculate the amount that should be added to value
      // for use in a numeric gradient calculation
    double getGradientEpsilon(double value, double scale, double minimum)
    {
      return std::max(minimum, fabs(value*scale));
    }

}


namespace vw {
namespace ORBA {

ORBAErrorEstimator::result_type
ORBAErrorEstimator::operator()(const ORBAErrorEstimator::domain_type& x) const
{
  return ProjectionError(x) + TrajectoryDependentErrors(x);
}

double ORBAErrorEstimator::TrajectoryDependentErrors(
    const ORBAErrorEstimator::domain_type& x) const
{
  double error = 0;

    // Prepare an appropriate trajectory calculator
  GravityAccelerationFunctor gravity(x.GM);
  TrajectoryCalculator traj_calc(gravity);

    // There needs to be one observation per timestamp
  assert(x.timestamps.size() == mObservations->getReadings().size());

    // Prepare to calculate the rest of the errors
  Vector3 prev_position = x.p0;
  Vector3 prev_velocity = x.v0;
  OrbitalReading::timestamp_t prev_t = x.timestamps[0];
  Vector3 next_position;
  Vector3 next_velocity;

    // Loop through each reading, calculate each error component
    // Keep a loop counter in sync too, for fast access to readings
  std::size_t i = 0;
  BOOST_FOREACH(OrbitalReading::timestamp_t time, x.timestamps)
  {
    error += getSingleTimestampError(i, x, traj_calc,
                                     prev_position, next_position,
                                     prev_velocity, next_velocity,
                                     prev_t, time);
    
      // Get ready for next point
    prev_position = next_position;
    prev_velocity = next_velocity;
    prev_t = time;
    i++;
  }

    // Now add in the point-independent error components
  error += pointIndependentSatelliteError(mWeights, x.precision_s)
      + pointIndependentRegistrationError(mWeights.size(), x.precision_r)
      + pointIndependentTimingError(mWeights, 1/x.precision_t);

  return error;
}
    

/*  
 * CalculateR
 *
 * calculates a 3x3 matrix of the orthonormal representation
 * of a point and its velocity. Orients the point around the
 * trajectory tangential to its orbit.
 *
 * TODO Check my explanation of CalculateR
 */
Matrix3x3 ORBAErrorEstimator::CalculateR(const Vector3& pos, const Vector3& vel) const
{
   double posNorm, velNorm, crossNorm;
   Vector3 posUnit, velUnit, crossUnit;
   Vector3 c;

   //Magnitude of position Vector
   posNorm = sqrt( (pos[0] * pos[0]) +
                   (pos[1] * pos[1]) +
                   (pos[2] * pos[2]));

   //Magnitutde of Velocity
   velNorm = sqrt( (vel[0] * vel[0]) +
                   (vel[1] * vel[1]) +
                   (vel[2] * vel[2]));

   //Cross product of Position x Velocity
   c[0] = (pos[1] * vel[2]) - (pos[2] * vel[1]);
   c[1] = (pos[2] * vel[0]) - (pos[0] * vel[2]);
   c[2] = (pos[0] * vel[1]) - (pos[1] * vel[0]);

   //Magnitude of the Cross product
   crossNorm = sqrt( (c[0] * c[0]) +
                     (c[1] * c[1]) +
                     (c[2] * c[2]));

   //Unit Vector of the position
   posUnit[0] = pos[0] / posNorm;
   posUnit[1] = pos[1] / posNorm;
   posUnit[2] = pos[2] / posNorm;

   //Unit Vector of the Velocity
   velUnit[0] = vel[0] / velNorm;
   velUnit[1] = vel[1] / velNorm;
   velUnit[2] = vel[2] / velNorm;
   
   //Unit vector fo the Cross Product
   crossUnit[0] = c[0] / crossNorm;
   crossUnit[1] = c[1] / crossNorm;
   crossUnit[2] = c[2] / crossNorm;
   
   //Matrix
   Matrix3x3 r;

   //Fill the Matrix
   r(0,0) = posUnit[0];
   r(0,1) = posUnit[1];
   r(0,2) = posUnit[2];

   r(1,0) = velUnit[0];
   r(1,1) = velUnit[1];
   r(1,2) = velUnit[2];

   r(2,0) = crossUnit[0];
   r(2,1) = crossUnit[1];
   r(2,2) = crossUnit[2];

   return r;
}

double ORBAErrorEstimator::ProjectionError(
    const ORBAErrorEstimator::domain_type& x) const
{
    // Loop through the control network
  double error = 0;
  BOOST_FOREACH(const ControlPoint& cp, *x.cnet)
  {
    error += getControlPointError(cp, x);
  }

    // Add in the point-independent projection error
  error += x.cnet->size()*prod(x.precision_p);
  
  return error;
}

double ORBAErrorEstimator::getControlPointError(
    const ControlPoint& cp,
    const ORBADecisionVariableSet& x) const
{
  double error = 0;
  
    // Get the estimated location of this control point, as stored
    // in the control network. This is a decision variable.
  Vector3 b = cp.position();
  
  BOOST_FOREACH(const ControlMeasure& cm, cp)
  {
      // Get the camera parameters for this camera
    const Vector3& a_location = x.pj[cm.image_id()];
    const Vector4& a_rotation = x.cj_second[cm.image_id()];
    
      // cm.position() is the pixel location of cp in this image.
      // It never changes.  We compare this actual pixel location
      // against the expected pixel location of world-space coordinate b.
    AdjustedCameraModel adj_cam(mObservations->getCamera(cm.image_id()),
                                a_location,
                                math::Quaternion<double>(a_rotation));
    Vector2 delta = elem_diff(cm.position(), adj_cam.point_to_pixel(b));
    error += sum(elem_prod(elem_prod(delta, delta),  x.precision_p));
  }
  return error;
}


/*
 * Point-Independent Registration Error
 * 
 * In:
 *  std::size_t point_count: Number of points in the orbit.
 *  Vector3 precisionsR : Calculated Precision from ORBA
 *
*/
double ORBAErrorEstimator::pointIndependentRegistrationError(
    std::size_t point_count, 
    const Vector3& precisionR) const
{
    // the minus is in lieu of inverting precision into covariance
  return  -point_count * log(prod(precisionR));
}

/*
 * Registration Error for a single reading
 * 
 * In:
 * Vector3& p   : Original Apollo reading (or is this supposed to be BA?)
 * Vector3& s   : Orbital Refined position
 * Matrix3x3& r : Calculated Radial coordinates from Camera positions.
 * Vector3& precisionsR : Calculated Precision from ORBA
 *
*/
double ORBAErrorEstimator::getSinglePointRegistrationError(
    const Vector3& p, 
    const Vector3& s,
    const Matrix3x3& r,
    const Vector3& precisionR) const
{
  Vector3 d = r*(p-s);
  Vector3 t = elem_prod(d, precisionR);

  return dot_prod(t, d);
}


/*
 * Point-Independent Satellite Error.
 * The portion of the error that doesn't change when
 * a point's location changes
 * 
 * In:
 *   const std::vector<double>& w: Weights for each point
 *   const Vector3& precisionS: Satellite precision
 *
*/
double ORBAErrorEstimator::pointIndependentSatelliteError(
    const std::vector<double>& w,
    const Vector3& precisionS) const
{
   //This is the log of the inverse precision
  return std::accumulate(w.begin(), w.end(), 0) // sum of weights
      * -log( prod(precisionS) );
}

/*
 * Satellite Error for a single point
 * 
 * In:
 * Vector3& q  : Original Apollo reading
 * Vector3& s  : Orbital Refined position
 * Matrix3x3 r : Calculated Radial coordinates from Camera positions.
 * double& w   : Weight of position from Orbital Refinement
 * Vector3& precisionsS : Calculated Precision from ORBA
 *
*/
double ORBAErrorEstimator::getSinglePointSatelliteError(
    const Vector3& q,
    const Vector3& s,
    const Matrix3x3& r,
    const double& w,
    const Vector3& precisionS) const
{
    //This is ej in Taemin's equation
  Vector3 d = r * (q - s);
  
    //t is now transpose e * inverse variance
  Vector3 t = elem_prod(d, precisionS);
  
  return w * dot_prod(t, d);
}


/*
 * Point-Independent TimingError
 *
 * double timeVariance           : variance from BA
 * Vector<double>& timeWeights   : weights for each point
 *
 * TODO Switch from variance to precision so it's consistent with the
 *      other elements and with how it's stored in the decision variable.
 */
double ORBAErrorEstimator::pointIndependentTimingError(
    const std::vector<double>& timeWeights,
    double timeVariance) const
{
  double weightSum = std::accumulate(timeWeights.begin(), timeWeights.end(), 0);
  
    //weight sum * ln (1/variance)
  return weightSum * -log(timeVariance);
}

/*
 * Timing Error for a single point
 *
 * double observed_time  : original time value
 * double estimated_time : OR-calculated time value
 * double timeVariance   : variance value
 * double weight         : weight for point
 *
 * TODO Deal with times using milliseconds instead of doubles.
 *
 * TODO Switch from variance to precision so it's consistent with the
 *      other elements and with how it's stored in the decision variable.
 */
double ORBAErrorEstimator::getSinglePointTimingError(
    OrbitalReading::timestamp_t observed_time,
    OrbitalReading::timestamp_t estimated_time,
    const double& time_variance,
    const double& weight) const
{
    // Because times are unsigned, we want to subtract the
    // bigger from the smaller time
  double time_diff = 0;
  if (observed_time > estimated_time)
    time_diff = observed_time - estimated_time;
  else
    time_diff = estimated_time - observed_time;
  
    // Convert to seconds
  time_diff /= 1000;
  
  return time_diff * time_diff * weight * time_variance;
}


ORBAGradientSet ORBAErrorEstimator::gradient(
    const ORBADecisionVariableSet& x_in ) const
{
    // Create a gradient set to pass back
  ORBAGradientSet gradient(x_in);

    // Create a mutable copy of x
  ORBADecisionVariableSet x = x_in;

    // We'll be calculating a forward gradient for each
    // element of x, one at a time

    // Start with those that affect all trajectory-related gradients
  double traj_error = TrajectoryDependentErrors(x);
  
  gradient.GM = calculateTrajectoryGradient(x.GM, 1e-7, x, traj_error);
  for (int i = 0; i < 3; ++i)
  {
    gradient.p0[i] = calculateTrajectoryGradient(x.p0[i], 1e-7, x, traj_error);
    gradient.v0[i] = calculateTrajectoryGradient(x.v0[i], 1e-7, x, traj_error);
  }

    // Now calculate gradients for each reading's timestamp.
    // For efficiency, we also calculate most of the precision gradients
    // at the same time.
  Vector3 precision_r_gradient;
  Vector3 precision_s_gradient;
  double precision_t_gradient;
  calculateTimeGradients(x, gradient, precision_r_gradient,
                         precision_s_gradient, precision_t_gradient);
  
    // The rest of the gradients go in x_k.
    // We'll keep a running index
  std::size_t i_x_k = 0;

    // Gradients for landmark locations,
    // one per 3D coord component
  BOOST_FOREACH(ControlPoint& cp, *x.cnet)
  {
      // Get the estimated location of this control point, as stored
      // in the control network. This is a decision variable.
    Vector3 b = cp.position();

      // get the error for this cp
    double cp_error = getControlPointError(cp, x);

      // Now tweak each dimension, get gradient
    for (int i = 0; i < 3; ++i)
    {
      double b_save = b[i];
      double epsilon = getGradientEpsilon(b_save, 1e-7, 1e-7);
      b[i] += epsilon;
      cp.set_position(b);
      double error_new = getControlPointError(cp, x);
      gradient.x_k[i_x_k++] = (error_new - cp_error) / epsilon;
      b[i] = b_save;
    }
    cp.set_position(b);
  }

    // Now go through pj.  Each camera can be referenced by
    // multiple control points, so we have to do the whole
    // projection error.
  double project_error = ProjectionError(x);
  
  BOOST_FOREACH(Vector3& pj, x.pj)
  {
    for (int i = 0; i < 3; ++i)
    {
      gradient.x_k[i_x_k++] =
          calculateProjectionGradient(pj[i], 1e-7, x, project_error);
    }
  }
    // cj_second
  BOOST_FOREACH(Vector4& cj, x.cj_second)
  {
    for (int i = 0; i < 4; ++i)
    {
      gradient.x_k[i_x_k++] =
          calculateProjectionGradient(cj[i], 1e-7, x, project_error);
    }
  }

    // precision values
  for (int i = 0; i < 2; i++)
  {
      // projection precision only affects projection error    
    gradient.x_k[i_x_k++] =
        calculateTrajectoryGradient(x.precision_p[i], 1e-7, x, traj_error);
  }
    // Store the previously-calculated precision gradients
  for (int i = 0; i < 3; i++)
  {
    gradient.x_k[i_x_k++] = precision_r_gradient[i];
  }
  for (int i = 0; i < 3; i++)
  {
    gradient.x_k[i_x_k++] = precision_s_gradient[i];
  }
  gradient.x_k[i_x_k++] = precision_t_gradient;
  
  return gradient;
}

/// Calculate a numeric gradient using forward finite difference.
/// \param to_tweak A reference to the value for which a gradient
///        is being calculated.
/// \param tweak_scaling The amount by which the variable should be
///        scaled when calculating the gradient.  Also serves as a
///        minimum amount by which the variable will be tweaked.
/// \param x The decision variable set to use in the calculation.
/// \param original_error The error before any tweaks were made.
double ORBAErrorEstimator::calculateTrajectoryGradient(
    double& to_tweak,
    double tweak_scaling,
    const ORBADecisionVariableSet& x,
    double original_trajectory_error) const
{
  double save = to_tweak;
  double eps = getGradientEpsilon(save, tweak_scaling, tweak_scaling);
  to_tweak += eps;
  double new_error = TrajectoryDependentErrors(x);
  to_tweak = save;
  return (new_error - original_trajectory_error) / eps;
}

unsigned ORBAErrorEstimator::dimension() const
{
  return mObservations->getReadings().size() * 8
      + mObservations->getControlNetwork()->size() * 3
      + 16;
}

// calculate the errors related to a given reading, given the reading's
// time value and the previous state.
// Returns the error, and sets next_position and next_velocity parameters
// to the reading's calculated p and v.
double ORBAErrorEstimator::getSingleTimestampError(
    std::size_t i, // Index of point
    const ORBADecisionVariableSet& x, // full set of variables, only a few are used
    const TrajectoryCalculator& traj_calc,
    const Vector3& prev_position, Vector3& next_position, // prev_* is [in]
    const Vector3& prev_velocity, Vector3& next_velocity, // next_* is [out]
    OrbitalReading::timestamp_t prev_t, // previous time
    OrbitalReading::timestamp_t next_t) const // time we're calculating error for
{
    // get the observed coordinates and time
  const OrbitalCameraReading& observation = mObservations->getReading(i);
  
    // Get the OR-calculated coordinates and velocity
  traj_calc.calculateNextPoint(prev_position, prev_velocity,
                               next_t - prev_t,
                               next_position, next_velocity);
  
    // Get the BA-calculated coordinates
  Vector3 ba_coord = mObservations->getCamera(i)->camera_center() + x.pj[i];
  
    // Calculate R
  Matrix3x3 r = CalculateR(next_position, next_velocity);
  
    // Calculate the error components
  double error = getSinglePointSatelliteError(observation.mCoord, next_position,
                                              r, mWeights[i], x.precision_s);
  error += getSinglePointRegistrationError(ba_coord, next_position,
                                           r, x.precision_r);
  error += getSinglePointTimingError(observation.mTime, next_t,
                                     x.precision_t, mWeights[i]);
  
  return error;
}

void ORBAErrorEstimator::calculateTimeGradients(
    ORBADecisionVariableSet& x,
    ORBAGradientSet& gradient,
    Vector3& precision_r_gradient,
    Vector3& precision_s_gradient,
    double& precision_t_gradient) const
{
    // Initialize some variables we'll be using to keep running totals
  Vector3 total_tweaked_precision_r_delta(0,0,0);
  Vector3 total_tweaked_precision_s_delta(0,0,0);
  double total_tweaked_precision_t_delta = 0;

    // Calculate tweaked values for our precisions
  Vector3 precision_r_eps, precision_r_tweaked;
  Vector3 precision_s_eps, precision_s_tweaked;
  for (std::size_t i = 0; i < 3; i++)
  {
    precision_r_eps[i] = getGradientEpsilon(x.precision_r[i], 1e-7, 1e-7);
    precision_r_tweaked[i] = x.precision_r[i] + precision_r_eps[i];
    precision_s_eps[i] = getGradientEpsilon(x.precision_s[i], 1e-7, 1e-7);
    precision_s_tweaked[i] = x.precision_s[i] + precision_s_eps[i];
  }
  double precision_t_eps = getGradientEpsilon(x.precision_t, 1e-7, 1e-7);
  double precision_t_tweaked = x.precision_t + precision_t_eps;

    // We'll need to calculate each timestamp's position.
    // Get ready to do that.
  GravityAccelerationFunctor gravity(x.GM);
  TrajectoryCalculator traj_calc(gravity);
  Vector3 prev_position = x.p0;
  Vector3 prev_velocity = x.v0;
  Vector3 cur_position;
  Vector3 cur_velocity;
  Vector3 tweaked_position;
  Vector3 tweaked_velocity;
  OrbitalReading::timestamp_t prev_t = x.timestamps[0];
      
  const OrbitalReading::timestamp_t TIMESTAMP_EPSILON = 2;

    // Go through the readings
  for (std::size_t i = 0; i < x.timestamps.size(); i++)
  {
      // Alias to time values
    OrbitalReading::timestamp_t& cur_t = x.timestamps[i];
    
      // get the observed coordinates and time
    const OrbitalCameraReading& observation = mObservations->getReading(i);
    
      // Get the BA-calculated coordinates
    Vector3 ba_coord = mObservations->getCamera(i)->camera_center() + x.pj[i];
    
      // Get the OR-calculated coordinates and velocity.
      // Make sure this is the last call in the loop to change
      // cur_position and cur_velocity.
    traj_calc.calculateNextPoint(prev_position, prev_velocity,
                                 cur_t - prev_t,
                                 cur_position, cur_velocity);
    
      // Calculate R
    Matrix3x3 r = CalculateR(cur_position, cur_velocity);
    
      // Calculate the error components
    double reg_error =
        getSinglePointRegistrationError(ba_coord, cur_position,
                                        r, x.precision_r);
    double sat_error =
        getSinglePointSatelliteError(observation.mCoord, cur_position,
                                     r, mWeights[i], x.precision_s);
    double time_error =
        getSinglePointTimingError(observation.mTime, cur_t,
                                  x.precision_t, mWeights[i]);
    
      // total error at the current value
    double cur_err = sat_error + reg_error + time_error;

      // Error delta with tweaked precision_s
    for (int j = 0; j < 3; j++)
    {
      // Error delta with tweaked precision_r
    total_tweaked_precision_r_delta[j] +=
        getSinglePointRegistrationError(ba_coord, cur_position,
                                        r, precision_r_tweaked[j])
        - reg_error;

     total_tweaked_precision_s_delta[j] +=
        getSinglePointSatelliteError(observation.mCoord, cur_position,
                                     r, mWeights[i], precision_s_tweaked[j])
        - sat_error;
    }

      // Error delta with tweaked precision_t
    total_tweaked_precision_t_delta +=
        getSinglePointTimingError(observation.mTime, cur_t,
                                  precision_t_tweaked, mWeights[i])
        - time_error;

      // Error when you tweak the time value.
    double time_tweaked_err =
        getSingleTimestampError(i, x, traj_calc,
                                prev_position, tweaked_position,
                                prev_velocity, tweaked_velocity,
                                prev_t, cur_t + TIMESTAMP_EPSILON);
    

      // Now that we've got all the data, calculate the time gradient
    gradient.t[i] = (time_tweaked_err - cur_err) / TIMESTAMP_EPSILON;
    
      // Get ready for next iteration through the loop
    prev_position = cur_position;
    prev_velocity = cur_velocity;
    prev_t = cur_t;
  }
    // First time gradient is always zero.
    // We still went through the loop for i=0 so that the precision gradients
    // would get calculated, so just change the first time gradient to zero here.
  gradient.t[0] = 0.0;

    // Now get the rest of the precision data, calculate precision gradients
  for (int i = 0; i < 3; i++)
  {
    double save = x.precision_s[i];
    x.precision_s[i] = precision_s_tweaked[i];
    total_tweaked_precision_s_delta[i] +=
        pointIndependentSatelliteError(mWeights, x.precision_s);
    x.precision_s[i] = save;
    total_tweaked_precision_s_delta[i] -=
        pointIndependentSatelliteError(mWeights, x.precision_s);
    precision_s_gradient[i] =
        total_tweaked_precision_s_delta[i] / precision_s_eps[i];
    
    save = x.precision_r[i];
    x.precision_r[i] = precision_r_tweaked[i];
    total_tweaked_precision_r_delta[i] +=
        pointIndependentRegistrationError(x.timestamps.size(), x.precision_r);
    x.precision_r[i] = save;
    total_tweaked_precision_r_delta[i] -=
        pointIndependentRegistrationError(x.timestamps.size(), x.precision_r);
    precision_r_gradient[i] =
        total_tweaked_precision_r_delta[i] / precision_r_eps[i];
  }
  total_tweaked_precision_t_delta +=
      pointIndependentTimingError(mWeights, precision_t_tweaked)
      - pointIndependentTimingError(mWeights, x.precision_t);
  precision_t_gradient = total_tweaked_precision_t_delta / precision_t_eps;
}

double ORBAErrorEstimator::calculateProjectionGradient(
    double& to_tweak,
    double tweak_scaling,
    const ORBADecisionVariableSet& x,
    double original_projection_error) const
{
  double save = to_tweak;
  double eps = getGradientEpsilon(save, tweak_scaling, tweak_scaling);
  to_tweak += eps;
  double new_error = ProjectionError(x);
  to_tweak = save;
  return (new_error - original_projection_error) / eps;
}


}}
