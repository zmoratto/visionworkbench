#ifndef WEIGHT_CALCULATOR_HPP
#define WEIGHT_CALCULATOR_HPP

#include <list>
#include <vw/Math/Vector.h>
#include <vw/orbital_refinement/OrbitalReading.hpp>

class WeightCalculator
{
public:

  

  void calculateWeights(const std::list<OrbitalReading>& observations,
                        const std::vector<vw::Vector3>& estimated_locations,
                        std::vector<double>& weights);

private:

    double standardDeviationOneDvector(
                           std::vector<double>& v,
                           double mean);

    void timeVarianceCalculator(
                        const std::list<OrbitalReading>& observations,
                        std::vector<double>& timeWeights);
};


#endif
