#include <vw/ORBA/ORBAErrorEstimator.hpp>

#include <cmath>

using namespace vw;

namespace vw {
namespace ORBA {
        

Matrix3x3 ORBAErrorEstimator::CalculateR(Vector3 pos, Vector3 vel)
{
   double posNorm, velNorm, crossNorm;
   Vector3 posUnit, velUnit, crossUnit;
   Vector3 c;

   posNorm = sqrt( (pos[0] * pos[0]) +
                   (pos[1] * pos[1]) +
                   (pos[2] * pos[2]));

   velNorm = sqrt( (vel[0] * vel[0]) +
                   (vel[1] * vel[1]) +
                   (vel[2] * vel[2]));

   c[0] = (pos[1] * vel[2]) - (pos[2] * vel[1]);
   c[1] = (pos[2] * vel[0]) - (pos[0] * vel[2]);
   c[2] = (pos[0] * vel[1]) - (pos[1] * vel[0]);

   crossNorm = sqrt( (c[0] * c[0]) +
                     (c[1] * c[1]) +
                     (c[2] * c[2]));

   posUnit[0] = pos[0] / posNorm;
   posUnit[1] = pos[1] / posNorm;
   posUnit[2] = pos[2] / posNorm;

   velUnit[0] = vel[0] / velNorm;
   velUnit[1] = vel[1] / velNorm;
   velUnit[2] = vel[2] / velNorm;
   
   crossUnit[0] = c[0] / crossNorm;
   crossUnit[1] = c[1] / crossNorm;
   crossUnit[2] = c[2] / crossNorm;
   
   Matrix3x3 r;

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

double ORBAErrorEstimator::RegistrationError(const std::vector<Vector3>& BA, 
                                             const std::vector<Vector3>& OR,
                                             const std::vector<Matrix3x3>& r,
                                             const Vector3& precisionR)
{
   Vector3 d, t;
   double error = 0;
   Vector3 ip;
   for(std::size_t i = 0; i < BA.size(); i++)
   {
      t[0] = BA[i][0] - OR[i][0];
      t[1] = BA[i][1] - OR[i][1];
      t[2] = BA[i][2] - OR[i][2];

      d[0] = t[0] * r[i](0,0) + t[1] * r[i](0,1) + t[2] * r[i](0,2);
      d[1] = t[0] * r[i](1,0) + t[1] * r[i](1,1) + t[2] * r[i](1,2);
      d[2] = t[0] * r[i](2,0) + t[1] * r[i](2,1) + t[2] * r[i](2,2);

      t[0] = d[0] * precisionR[0];
      t[1] = d[1] * precisionR[1];
      t[2] = d[2] * precisionR[2];

      error += t[0] * d[0] + t[1] * d[1] + t[2] * d[2];
   }

   ip[0] = precisionR[1] * precisionR[2];
   ip[1] = precisionR[0] * precisionR[2];
   ip[2] = precisionR[0] * precisionR[1];

   //This is wrong, need to inverse the matrix first.
   // Also check indices...
   error += BA.size() * log( (ip[1] * ip[2] +
                              ip[0] * ip[2] +
                              ip[0] * ip[1]));
   return error;                                 
}

//AP is the original satellite positions
//OR is the orbital refined positions
double ORBAErrorEstimator::SatelliteError(std::vector<Vector3>& AP,
                                          std::vector<Vector3>& OR,
                                          std::vector<Matrix3x3>& r,
                                          Vector<double> w,
                                          Vector3 precisionS)
{
   Vector3 d, t;
   double error = 0;
   Vector3 ip;
   for(std::size_t i = 0; i < AP.size(); i++)
   {
      t[0] = AP[i][0] - OR[i][0];
      t[1] = AP[i][1] - OR[i][1];
      t[2] = AP[i][2] - OR[i][2];

      d[0] = t[0] * r[i](0,0) + t[1] * r[i](0,1) + t[2] * r[i](0,2);
      d[1] = t[0] * r[i](1,0) + t[1] * r[i](1,1) + t[2] * r[i](1,2);
      d[2] = t[0] * r[i](2,0) + t[1] * r[i](2,1) + t[2] * r[i](2,2);

      t[0] = d[0] * precisionS[0];
      t[1] = d[1] * precisionS[1];
      t[2] = d[2] * precisionS[2];

      error += (t[0] * d[0] + t[1] * d[1] + t[2] * d[2]) * w[i];
   }

   ip[0] = precisionS[1] * precisionS[2];
   ip[1] = precisionS[0] * precisionS[2];
   ip[2] = precisionS[0] * precisionS[1];

   //This is wrong, need to inverse the matrix first
   error += AP.size() * sum(w) * log( (ip[1] * ip[2] +
                                        ip[0] * ip[2] +
                                        ip[0] * ip[1]));
   return error;
}

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
