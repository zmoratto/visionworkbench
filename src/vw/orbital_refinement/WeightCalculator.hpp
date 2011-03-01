#ifndef WEIGHT_CALCULATOR_HPP
#define WEIGHT_CALCULATOR_HPP

class WeightCalculator
{
public:
    // empty placeholder, does nothing yet.
  void calculateWeights(const std::list<OrbitalReading>& observations,
                        const std::vector<vw::Vector3>& estimated_locations,
                        std::vector<bool>& weights)
      {}
};


#endif
