#ifndef WEIGHT_CALCULATOR_HPP
#define WEIGHT_CALCULATOR_HPP

#include <list>
#include <vw/Math/Vector.h>
#include <vw/orbital_refinement/OrbitalReading.hpp>

class WeightCalculator
{
public:

  

  void calculateWeights(std::list<OrbitalReading>& observations,
                        const std::vector<vw::Vector3>& estimated_locations,
                        std::vector<double>& weights);

  double smart_weighted_mean(std::vector<double>& weights,
                           std::vector<double> const & samples,
                           float const sign_level,
                           float const learn_rate,
                           float const error_tol, 
                           long const max_iter);

private:

    double standardDeviationOneDvector(
                           std::vector<double>& v,
                           double mean);

    void timeVarianceCalculator(
                        const std::list<OrbitalReading>& observations,
                        std::vector<double>& timeWeights);
                     
   vw::Vector3 Unit(vw::Vector3& v);                     
};


#endif
