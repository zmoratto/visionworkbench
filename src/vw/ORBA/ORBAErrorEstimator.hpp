#ifndef __VW_ORBA_ERROR_ESTIMATOR_HPP
#define __VW_ORBA_ERROR_ESTIMATOR_HPP

#include <vw/ORBA/ORBADecisionVariableSet.hpp>
#include <vw/ORBA/ORBAGradientSet.hpp>
#include <vw/ORBA/ObservationSet.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>

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

    // How error functions are nested:
    // operator()
    //   ProjectionError(x)
    //     getControlPointError(cp) [Once per ControlPoint]
    //   TrajectoryDependentErrors(x)
    //     getSingleTimestampError(p0, v0, t0, t[i]) [Once per reading]
    //       getSinglePointSatelliteError(p[i])
    //       getSinglePointRegistrationError(p[i])
    //       getSinglePointTimingError(t[i])      
    //     pointIndependentSatelliteError()
    //     pointIndependentRegistrationError()
    //     pointIndependentTimingError()

  double ProjectionError(const domain_type& x) const;
  
  double getControlPointError(
      const ControlPoint& cp,
      const ORBADecisionVariableSet& x) const;
  
  double TrajectoryDependentErrors(const domain_type& x) const;

  double getSingleTimestampError(
      std::size_t i,
      const ORBADecisionVariableSet& x,
      const TrajectoryCalculator& traj_calc,
      const Vector3& prev_position, Vector3& next_position,
      const Vector3& prev_velocity, Vector3& next_velocity,
      OrbitalReading::timestamp_t prev_t,
      OrbitalReading::timestamp_t next_t) const;

  double getSinglePointSatelliteError(const Vector3& original_reading,
                                      const Vector3& OR_refined,
                                      const Matrix3x3& r,
                                      const double& weight,
                                      const Vector3& precisionS) const;
  
  double getSinglePointRegistrationError(
      const Vector3& BA,
      const Vector3& OR,
      const Matrix3x3& r,
      const Vector3& precisionR) const;
  
  double getSinglePointTimingError(OrbitalReading::timestamp_t observed_time,
                                   OrbitalReading::timestamp_t estimated_time,
                                   const double& timeVariance,
                                   const double& weight) const;

  double pointIndependentSatelliteError(const std::vector<double>& weights,
                                        const Vector3& precisionS) const;

  double pointIndependentRegistrationError(
      std::size_t point_count, 
      const Vector3& precisionR) const;

  double pointIndependentTimingError(const std::vector<double>& timeWeights,
                                     double timeVariance) const;

    // Gradients are calculated using the relevant error functions
    // for each gradient component.  Some sub-functions have been split out
    // for readability and code reuse.
  double calculateTrajectoryGradient(double& to_tweak,
                                     double tweak_scaling,
                                     const ORBADecisionVariableSet& x,
                                     double original_trajectory_error) const;

  double calculateProjectionGradient(double& to_tweak,
                                     double tweak_scaling,
                                     const ORBADecisionVariableSet& x,
                                     double original_projection_error) const;

  void calculateTimeGradients(ORBADecisionVariableSet& x,
                              ORBAGradientSet& gradients,
                              Vector3& precision_r_gradient,
                              Vector3& precision_s_gradient,
                              double& precision_t_gradient) const;
  
  Matrix3x3 CalculateR(const Vector3& pos, const Vector3& vel) const;
  
  boost::shared_ptr<ObservationSet> mObservations;
  
    // The relative weight of each point in OR.
    // We hold a const reference to the weight vector,
    // which is updated elsewhere.
  const std::vector<double>& mWeights;
};

}} // namespace vw::ORBA


#endif
