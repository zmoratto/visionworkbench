#include <vw/ORBA/ORBAErrorEstimator.hpp>
#include <vw/Camera/CameraModel.h>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>

#include <boost/foreach.hpp>
#include <cmath>
#include <numeric>

using namespace vw;

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
// These are temporary!!! Until we unroll the error calculations
  std::vector<Vector3> p;
  std::vector<Vector3> q;
  std::vector<Matrix3x3> r;
  std::vector<Vector3> s;
  std::vector<double> t_calculated;
  std::vector<double> t_observed;
  Vector<double> w;
  
  double error = 0;

    // Prepare an appropriate trajectory calculator
  GravityAccelerationFunctor gravity(x.trajectory.GM);
  TrajectoryCalculator traj_calc(gravity);

    // There needs to be one observation per timestamp
  assert(x.trajectory.timestamps.size() == mObservations->getReadings().size());

    // Prepare to calculate the rest of the errors
  Vector3 prev_position = x.trajectory.p0;
  Vector3 prev_velocity = x.trajectory.v0;
  OrbitalReading::timestamp_t prev_t = x.trajectory.timestamps[0];
  Vector3 next_position;
  Vector3 next_velocity;

    // Loop through each reading, calculate each error component
    // Keep a loop counter in sync too, for fast access to readings
  std::size_t i = 0;
  BOOST_FOREACH(OrbitalReading::timestamp_t time, x.trajectory.timestamps)
  {
      // get the observed coordinates and time
    const OrbitalCameraReading& observation = mObservations->getReading(i);
    
      // Get the OR-calculated coordinates and velocity
    traj_calc.calculateNextPoint(prev_position, prev_velocity,
                                 time - prev_t,
                                 next_position, next_velocity);

      // this is temporary!!!
      // For now, store the OR-calculated position and velocity, then calculate
      // errors after we have all the data.  Later, we'll calculate the error for
      // a single point right here and NOT save the p and v.
    s.push_back(next_position);
    r.push_back(CalculateR(next_position, next_velocity));
      // One of the following two is wrong...it should be
      // the time as estimated by BA.  Replace with the following:
      // whichever.push_back(x.pj[i] + observation.mCamera.position());
    p.push_back(observation.mCoord);
    q.push_back(observation.mCoord);
      // Need to get weights...
//    w.push_back();
    t_calculated.push_back((double)(time) / 1000);
    t_observed.push_back((double)(observation.mTime) / 1000);

      // Get ready for next point
    prev_position = next_position;
    prev_velocity = next_velocity;
    prev_t = time;
    i++;
  }

    // this is temporary!!!
  error += SatelliteError(q, s, r, mWeights, x.precision_s)
      + RegistrationError(p, s, r, x.precision_r)
      + TimingError(t_observed, t_calculated, 1/x.precision_t, mWeights);
  
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
  BOOST_FOREACH(const ControlPoint& cp, x.cnet)
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
 * Registration Error
 * 
 * In:
 * vector<Vector3>& p  : Original Apollo readings
 * vector<Vector3>& s  : Orbital Refined positions
 * vector<Matrix3x3> r : Calculated Radial coordinates from Camera positions.
 *                       This should be calculated separately as part of an
 *                       initialization routine.
 * Vector3 precisionsR : Calculated Precision from ORBA
 *
 * TODO make things const, maybe pass w by reference
 *      optimize?
*/
double ORBAErrorEstimator::RegistrationError(const std::vector<Vector3>& p, 
                                             const std::vector<Vector3>& s,
                                             const std::vector<Matrix3x3>& r,
                                             const Vector3& precisionR) const
{
   Vector3 d, t; //d and t are temp vectors for math
   double error = 0;
   for(std::size_t i = 0; i < p.size(); i++)
   {
      t[0] = p[i][0] - s[i][0];
      t[1] = p[i][1] - s[i][1];
      t[2] = p[i][2] - s[i][2];

      d[0] = t[0] * r[i](0,0) + t[1] * r[i](0,1) + t[2] * r[i](0,2);
      d[1] = t[0] * r[i](1,0) + t[1] * r[i](1,1) + t[2] * r[i](1,2);
      d[2] = t[0] * r[i](2,0) + t[1] * r[i](2,1) + t[2] * r[i](2,2);

      t[0] = d[0] * ( 1.0 / precisionR[0]);
      t[1] = d[1] * ( 1.0 / precisionR[1]);
      t[2] = d[2] * ( 1.0 / precisionR[2]);

      error += t[0] * d[0] + t[1] * d[1] + t[2] * d[2];
   }

   // Also check indices...
   error -= p.size() * log( precisionR[0] * precisionR[1] * precisionR[2]);
   return error;                                 
}

/*
 * Satellite Error
 * 
 * In:
 * vector<Vector3>& q  : Original Apollo readings
 * vector<Vector3>& s  : Orbital Refined positions
 * vector<Matrix3x3> r : Calculated Radial coordinates from Camera positions.
 *                       This should be calculated separately as part of an
 *                       initialization routine.
 * Vector<double> w    : Weights of each position from Orbital Refinement
 * Vector3 precisionsS : Calculated Precision from ORBA
 *
 * TODO make things const, maybe pass w by reference
 *      optimize?
*/
double ORBAErrorEstimator::SatelliteError(const std::vector<Vector3>& q,
                                          const std::vector<Vector3>& s,
                                          const std::vector<Matrix3x3>& r,
                                          const std::vector<double>& w,
                                          const Vector3& precisionS) const
{
   Vector3 d, t; //temp vectors for math. T is for temp. D is for . . .temp.
   double error = 0;
   for(std::size_t i = 0; i < q.size(); i++)
   {
      //t is the delta between original reading/OR values
      t[0] = q[i][0] - s[i][0];
      t[1] = q[i][1] - s[i][1];
      t[2] = q[i][2] - s[i][2];

      //This is R * t, ej in Taemin's equation
      d[0] = t[0] * r[i](0,0) + t[1] * r[i](0,1) + t[2] * r[i](0,2);
      d[1] = t[0] * r[i](1,0) + t[1] * r[i](1,1) + t[2] * r[i](1,2);
      d[2] = t[0] * r[i](2,0) + t[1] * r[i](2,1) + t[2] * r[i](2,2);

      //t is now transpose e * inverse variance
      t[0] = d[0] * ( 1.0 / precisionS[0]);
      t[1] = d[1] * ( 1.0 / precisionS[1]);
      t[2] = d[2] * ( 1.0 / precisionS[2]);

      //adding the previous value with 
      error += (t[0] * d[0] + t[1] * d[1] + t[2] * d[2]) * w[i];
   }

   //This is the log of the inverse precision
   error -= q.size() * std::accumulate(w.begin(), w.end(), 0) * log( precisionS[0] * precisionS[1] * precisionS[2] );
   return error;
}


/*
 * TimingError
 *
 * Vector<double>& timeValues    : original time values
 * Vector<double>& timeEstimates : OR time values
 * double timeVariance           : variance from BA
 * Vector<double>& timeWeights   : weights for each point
 *
 * TODO Check if the weight is for the point or just the time component
 * of the weight.
 *
 * TODO Deal with times using milliseconds instead of doubles.
 *
 * TODO Switch from variance to precision so it's consistent with the
 *      other elements and with how it's stored in the decision variable.
 */
double ORBAErrorEstimator::TimingError(const std::vector<double>& timeValues,
                                       const std::vector<double>& timeEstimates,
                                       double timeVariance,
                                       const std::vector<double>& timeWeights) const
{
   double diff = 0;
   //Sum weights
   double weightSum = 0;
   for(std::size_t i = 0; i < timeWeights.size(); i++)
   {
      weightSum += timeWeights[i];
   }

   //square variance
   double sigma2 = timeVariance * timeVariance;

   //weight sum * ln (varaince squared)
   double error = weightSum * (log (sigma2));

   //sum of -
   // weight(j) * (Real time - estimate time)^2 / var^2
   for(std::size_t i = 0; i < timeValues.size(); i++)
   {
      diff +=( (timeValues[i] - timeEstimates[i])* (timeValues[i] - timeEstimates[i]) * timeWeights[i] ) / sigma2;
   }

   return error + diff;
}

   
}}
