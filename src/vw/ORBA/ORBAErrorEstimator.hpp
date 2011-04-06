#ifndef __VW_ORBA_ERROR_ESTIMATOR_HPP
#define __VW_ORBA_ERROR_ESTIMATOR_HPP

#include <vw/ORBA/ORBADecisionVariableSet.hpp>
#include <vw/ORBA/ORBAGradientSet.hpp>

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vector>

namespace vw {
namespace ORBA {

using namespace vw;
    
class ORBAErrorEstimator
{
public:

// This section is all required for use in CG
  typedef ORBADecisionVariableSet domain_type;
  typedef ORBAGradientSet gradient_type;
  typedef double result_type;

  /// Evaluate the error for the given set of decision variables.
  result_type operator()(const domain_type& x) const;

  /// Evaluate the gradient for the given set of decision variables.
  gradient_type gradient( domain_type const& x ) const;

  /// the dimension of the gradient vector.
  unsigned dimension() const;

// End of what's required for CG
  
private:
  Matrix3x3 CalculateR(Vector3 pos, Vector3 vel) const;
  
  double ProjectionError(const domain_type& x) const;
  
  double RegistrationError(
      const std::vector<Vector3>& BA,
      const std::vector<Vector3>& OR,
      const std::vector<Matrix3x3>& r,
      const Vector3& precisionR) const;
  
  double SatelliteError(std::vector<Vector3>& AP,
                        std::vector<Vector3>& OR,
                        std::vector<Matrix3x3>& r,
                        Vector<double> w,
                        Vector3 precisionS) const;

  double TimingError(const Vector<double>& timeValues,
                     const Vector<double>& timeEstimates,
                     double timeVariance,
                     const Vector<double>& timeWeights) const;
  
};

}} // namespace vw::ORBA


#endif
