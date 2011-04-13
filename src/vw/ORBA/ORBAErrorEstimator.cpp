#include <vw/ORBA/ORBAErrorEstimator.hpp>
#include <vw/Camera/CameraModel.h>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>

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
      // get the observed coordinates and time
    const OrbitalCameraReading& observation = mObservations->getReading(i);
    
      // Get the OR-calculated coordinates and velocity
    traj_calc.calculateNextPoint(prev_position, prev_velocity,
                                 time - prev_t,
                                 next_position, next_velocity);

      // Get the BA-calculated coordinates
    Vector3 ba_coord = mObservations->getCamera(i)->camera_center() + x.pj[i];
    
      // Calculate R
    Matrix3x3 r = CalculateR(next_position, next_velocity);

      // Calculate the error components
    error += getSinglePointSatelliteError(observation.mCoord, next_position,
                                          r, mWeights[i], x.precision_s);
    error += getSinglePointRegistrationError(ba_coord, next_position,
                                        r, x.precision_r);
    error += getSinglePointTimingError(observation.mTime, time,
                                       x.precision_t, mWeights[i]);

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
      // Get the estimated location of this control point, as stored
      // in the control network. This is a decision variable.
    Vector3 b = cp.position();
    
      // Loop through each measure in the control point
    BOOST_FOREACH(const ControlMeasure& cm, cp)
    {
        // To do:  We need an ORBAModel to store our observations.
        // We need to get the camera from this model.
      
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
   return  -point_count * log( precisionR[0] * precisionR[1] * precisionR[2]);
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
  Vector3 t = elem_quot(d, precisionR);

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
  return -w.size()
      * std::accumulate(w.begin(), w.end(), 0) // sum of weights
      * log( precisionS[0] * precisionS[1] * precisionS[2] );
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
  Vector3 t = elem_quot(d, precisionS);
  
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
  
    //square variance
  double sigma2 = timeVariance * timeVariance;
  
    //weight sum * ln (varaince squared)
  return weightSum * (log (sigma2));
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
  
  return time_diff * time_diff * weight / (time_variance*time_variance);
}


ORBAGradientSet ORBAErrorEstimator::gradient(
    const ORBADecisionVariableSet& x_in ) const
{
    // Get the error right @ x
  double cur_error = operator()(x_in);

    // Create a gradient set to pass back
  ORBAGradientSet gradient(x_in);

    // Create a mutable copy of x
  ORBADecisionVariableSet x = x_in;

    // Get a forward gradient for all the elements of x, one at a time
  gradient.GM = calculateGradient(x.GM, 1e-7, x, cur_error);
  for (int i = 0; i < 3; ++i)
  {
    gradient.p0[i] = calculateGradient(x.p0[i], 1e-7, x, cur_error);
    gradient.v0[i] = calculateGradient(x.v0[i], 1e-7, x, cur_error);
  }

    // Optimization idea, only recalculate the portion of the error relevant
    // to each timestamp.
  for (std::size_t i = 0; i < x.timestamps.size(); i++)
  {
      // Because times are fixed precision, we use a fixed epsilon.
      // Put gradient calculation right here instead of calling the
      // function used for other variables.
    const OrbitalReading::timestamp_t eps = 2; // Fixed epsilon of 2 ms.
    x.timestamps[i] += eps;
    double error_new = operator()(x);
    gradient.t[i] = (error_new - cur_error) / (eps);
    x.timestamps[i] -= eps;
  }

    // The rest of the gradients go in x_k.
    // We'll keep a running index
  std::size_t i_x_k = 0;

    // Gradients for landmark locations
  BOOST_FOREACH(ControlPoint& cp, *x.cnet)
  {
      // Get the estimated location of this control point, as stored
      // in the control network. This is a decision variable.
    Vector3 b = cp.position();
    for (int i = 0; i < 3; ++i)
    {
        // We can't use the same function for the gradient calculation
        // because we don't have a way to get a reference to the
        // position vector we're working on.  Essentially repeat
        // the gradient calculation code here.
      double b_save = b[i];
      double epsilon = getGradientEpsilon(b_save, 1e-7, 1e-7);
      b[i] += epsilon;
      cp.set_position(b);
      double error_new = operator()(x);
      gradient.x_k[i_x_k++] = (error_new - cur_error) / (epsilon);
      b[i] = b_save;
    }
    cp.set_position(b);
  }

    // Now go through pj
  BOOST_FOREACH(Vector3& pj, x.pj)
  {
    for (int i = 0; i < 3; ++i)
    {
      gradient.x_k[i_x_k++] =
          calculateGradient(pj[i], 1e-7, x, cur_error);
    }
  }
  
    // cj_second
  BOOST_FOREACH(Vector4& cj, x.cj_second)
  {
    for (int i = 0; i < 4; ++i)
    {
      gradient.x_k[i_x_k++] =
          calculateGradient(cj[i], 1e-7, x, cur_error);
    }
  }

    // precision values
  for (int i = 0; i < 2; i++)
  {
    gradient.x_k[i_x_k++] =
        calculateGradient(x.precision_p[i], 1e-7, x, cur_error);
  }
  for (int i = 0; i < 3; i++)
  {
    gradient.x_k[i_x_k++] =
        calculateGradient(x.precision_r[i], 1e-7, x, cur_error);
  }
  for (int i = 0; i < 3; i++)
  {
    gradient.x_k[i_x_k++] =
        calculateGradient(x.precision_s[i], 1e-7, x, cur_error);
  }
  gradient.x_k[i_x_k++] =
      calculateGradient(x.precision_t, 1e-7, x, cur_error);
  
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
double ORBAErrorEstimator::calculateGradient(
    double& to_tweak,
    double tweak_scaling,
    const ORBADecisionVariableSet& x,
    double original_error) const
{
  double save = to_tweak;
  double eps = getGradientEpsilon(save, tweak_scaling, tweak_scaling);
  to_tweak += eps;
  double new_error = operator()(x);
  to_tweak = save;
  return (new_error - original_error) / eps;
}

unsigned ORBAErrorEstimator::dimension() const
{
  return mObservations->getReadings().size() * 8
      + mObservations->getControlNetwork()->size() * 3
      + 16;
}

}}
