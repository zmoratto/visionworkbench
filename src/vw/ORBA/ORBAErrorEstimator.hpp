#ifndef ORBA_ERROR_ESTIMATOR_HPP
#define ORBA_ERROR_ESTIMATOR_HPP

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vector>

namespace vw {
namespace ORBA {

using namespace vw;
    
class ORBAErrorEstimator
{
private:
  Matrix3x3 CalculateR(Vector3 pos, Vector3 vel);
  
  double ProjectionError();
  
  double RegistrationError(
      const std::vector<Vector3>& BA,
      const std::vector<Vector3>& OR,
      const std::vector<Matrix3x3>& r,
      const Vector3& precisionR);
  
  double SatelliteError(std::vector<Vector3>& AP,
                        std::vector<Vector3>& OR,
                        std::vector<Matrix3x3>& r,
                        Vector<double> w,
                        Vector3 precisionS);

  double TimingError(const Vector<double>& timeValues,
                     const Vector<double>& timeEstimates,
                     double timeVariance,
                     const Vector<double>& timeWeights);
  
};

}} // namespace vw::ORBA


#endif
