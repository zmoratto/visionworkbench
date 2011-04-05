#include <vw/ORBA/ORBAErrorEstimator.hpp>

#include <cmath>

using namespace vw;

namespace vw {
namespace ORBA {
        
/*  
 * CalculateR
 *
 * calculates a 3x3 matrix of the orthonormal representation
 * of a point and its velocity. Orients the point around the
 * trajectory tangential to its orbit.
 *
 * TODO Check my explanation of CalculateR
 */
Matrix3x3 ORBAErrorEstimator::CalculateR(Vector3 pos, Vector3 vel)
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

double ORBAErrorEstimator::ProjectionError()
{
    // To do.
  return 0;
}


/*
 * Registration Error
 * 
 * In:
 * vector<Vector3>& p  : Original Apollo readings
 * vector<Vector3>& s  : Orbital Refinemed positions
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
                                             const Vector3& precisionR)
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
 * vector<Vector3>& s  : Orbital Refinemed positions
 * vector<Matrix3x3> r : Calculated Radial coordinates from Camera positions.
 *                       This should be calculated separately as part of an
 *                       initialization routine.
 * Vector<double> w    : Weights of each position from Orbital Refinement
 * Vector3 precisionsS : Calculated Precision from ORBA
 *
 * TODO make things const, maybe pass w by reference
 *      optimize?
*/
double ORBAErrorEstimator::SatelliteError(std::vector<Vector3>& q,
                                          std::vector<Vector3>& s,
                                          std::vector<Matrix3x3>& r,
                                          Vector<double> w,
                                          Vector3 precisionS)
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
   error -= q.size() * sum(w) * log( precisionS[0] * precisionS[1] * precisionS[2] );
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
 */
double ORBAErrorEstimator::TimingError(const Vector<double>& timeValues,
                                       const Vector<double>& timeEstimates,
                                       double timeVariance,
                                       const Vector<double>& timeWeights)
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
   double error = weightSum * (std::log (sigma2));

   //sum of -
   // weight(j) * (Real time - estimate time)^2 / var^2
   for(std::size_t i = 0; i < timeValues.size(); i++)
   {
      diff +=( (timeValues[i] - timeEstimates[i])* (timeValues[i] - timeEstimates[i]) * timeWeights[i] ) / sigma2;
   }

   return error + diff;
}

   
}}
