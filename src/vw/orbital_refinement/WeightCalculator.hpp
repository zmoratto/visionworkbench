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
                             const std::vector<double>& samples,
                             const double sign_level,
                             const double learn_rate,
                             const double error_tol, 
                             const long max_iter);

private:

    double standardDeviationOneDvector(
                           std::vector<double>& v,
                           double mean);

    void timeVarianceCalculator(
                        const std::list<OrbitalReading>& observations,
                        std::vector<double>& timeWeights);
                     
   double norm(vw::Vector3& v);                     
};


#endif
