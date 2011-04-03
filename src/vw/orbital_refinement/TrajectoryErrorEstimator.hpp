#ifndef TRAJECTORY_ERROR_ESTIMATOR_HPP
#define TRAJECTORY_ERROR_ESTIMATOR_HPP

#include <list>
#include <vector>
#include <vw/orbital_refinement/OrbitalReading.hpp>
#include <vw/orbital_refinement/TrajectoryDecisionVariableSet.hpp>
#include <vw/orbital_refinement/TrajectoryGradientSet.hpp>

/// Calculates the weighted error of a set of values compared to
/// a set of observations.  Compatible with the VW conjugate gradient
/// implementation.
class TrajectoryErrorEstimator
{
public:
  typedef double result_type;
  typedef TrajectoryDecisionVariableSet domain_type;
  typedef TrajectoryGradientSet gradient_type;

  /// Constructor.
  /// The error estimator will hold a reference to the passed-in
  /// list of observations.  It must not be destroyed or allowed to
  /// go out of scope until after this error estimator is done being used.
  TrajectoryErrorEstimator(const std::list<OrbitalReading>& observations);

  /// The error estimator will hold a reference to the passed-in
  /// list of observations and the passed in list of weights.
  /// Neither must not be destroyed or allowed to
  /// go out of scope until after this error estimator is done being used.
  TrajectoryErrorEstimator(const std::list<OrbitalReading>& observations,
                           const std::vector<double>& weights);

  /// Evaluate the error for the given set of decision variables.
  result_type operator()(const domain_type& x) const;

  /// Evaluate the gradient for the given set of decision variables.
  gradient_type gradient( domain_type const& x ) const;

  /// the dimension of the gradient vector.
  unsigned dimension() const;

private:
  double calculateError(double GM, vw::Vector3 p0, vw::Vector3 v0,
                        const std::vector<OrbitalReading::timestamp_t>& t,
                        bool calculate_gradient);
  
  double calculatePositionError(std::size_t index,
                                const vw::Vector3& calculated_position,
                                const vw::Vector3& calculated_velocity,
                                const OrbitalReading& observation,
                                bool calculate_gradient);
  
  double forwardFiniteDifferenceGradient(
      double& to_tweak, double epsilon,
      double& GM, vw::Vector3& p0, vw::Vector3& v0,
      const std::vector<OrbitalReading::timestamp_t>& t, double old_error) const;

  double centralFiniteDifferenceGradient(
      double& to_tweak, double epsilon,
      double& GM, vw::Vector3& p0, vw::Vector3& v0,
      const std::vector<OrbitalReading::timestamp_t>& t) const;

    // Just sits there empty unless no weights are passed in the constructor.
  std::vector<double> _default_weights;
    // The original coordinates against which errors are calculated.
  const std::list<OrbitalReading>& _observations;
    // The relative weight of each point
  const std::vector<double>& _weights;
    // The error of the most recently evaluated domain vector
  double _cached_error;
    // The error gradient for the most recently evaluated domain vector
  TrajectoryGradientSet _gradient;
};

#endif
