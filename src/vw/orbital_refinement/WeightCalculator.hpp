#ifndef WEIGHT_CALCULATOR_HPP
#define WEIGHT_CALCULATOR_HPP

class WeightCalculator
{
public:

  

  void calculateWeights(const std::list<OrbitalReading>& observations,
                        const std::vector<vw::Vector3>& estimated_locations,
                        std::vector<bool>& weights);

private:

    double standardDeviationOneDvector(
                           std::vector<double>& v,
                           double mean);

    void timeVarianceCalcuator(
                        const std::list<OrbitalReading>& observations,
                        std::vector<double>& timeWeights);
};


#endif
