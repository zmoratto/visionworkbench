#ifndef __VW_ORBA_ERROR_ESTIMATOR_HPP
#define __VW_ORBA_ERROR_ESTIMATOR_HPP

#include <vw/ORBA/ORBADecisionVariableSet.hpp>
#include <vw/ORBA/ORBAGradientSet.hpp>
#include <vw/ORBA/ObservationSet.hpp>

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vector>

namespace vw {
namespace ORBA {

using namespace vw;
    
class ORBAErrorEstimator
{
public:

    // Constructor.
    // Takes the readings we'll be compared against as input.
  ORBAErrorEstimator(boost::shared_ptr<ObservationSet> observations,
                     const std::vector<double>& weights) 
          : mObservations(observations),
            mWeights(weights)
  {}

// This section is all required for use in CG
  typedef ORBADecisionVariableSet domain_type;
  typedef ORBAGradientSet gradient_type;
  typedef double result_type;

  /// Evaluate the error for the given set of decision variables.
  result_type operator()(const domain_type& x) const;

  /// Evaluate the gradient for the given set of decision variables.
  gradient_type gradient(const domain_type& x) const;

  /// the dimension of the gradient vector.
  unsigned dimension() const;

// End of what's required for CG
  
private:

  double TrajectoryDependentErrors(const domain_type& x) const;

  Matrix3x3 CalculateR(const Vector3& pos, const Vector3& vel) const;
  
  double ProjectionError(const domain_type& x) const;
  
  double RegistrationError(
      const std::vector<Vector3>& BA,
      const std::vector<Vector3>& OR,
      const std::vector<Matrix3x3>& r,
      const Vector3& precisionR) const;
  
  double SatelliteError(const std::vector<Vector3>& AP,
                        const std::vector<Vector3>& OR,
                        const std::vector<Matrix3x3>& r,
                        const std::vector<double>& w,
                        const Vector3& precisionS) const;

  double TimingError(const std::vector<double>& timeValues,
                     const std::vector<double>& timeEstimates,
                     double timeVariance,
                     const std::vector<double>& timeWeights) const;

  double calculateGradient(double& to_tweak,
                           double tweak_scaling,
                           const ORBADecisionVariableSet& x,
                           double original_error) const;
  
  boost::shared_ptr<ObservationSet> mObservations;
  
    // The relative weight of each point in OR.
    // We hold a const reference to the weight vector,
    // which is updated elsewhere.
  const std::vector<double>& mWeights;
};

}} // namespace vw::ORBA


#endif
