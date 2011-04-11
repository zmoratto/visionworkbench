#ifndef WEIGHT_CALCULATOR_HPP
#define WEIGHT_CALCULATOR_HPP

#include <list>
#include <vw/Math/Vector.h>
#include <vw/orbital_refinement/OrbitalReading.hpp>

class WeightCalculator
{
public:

  template <typename ContainerT>
  void calculateWeights(const ContainerT& observations,
                        const std::vector<vw::Vector3>& estimated_locations,
                        std::vector<double>& weights);

  double smart_weighted_mean(std::vector<double>& weights,
                             const std::vector<double>& timeWeights,
                             double tAvg,
                             const std::vector<double>& samples,
                             const double sign_level,
                             const double learn_rate,
                             const double error_tol, 
                             const long max_iter);

private:

  double standardDeviationOneDvector(
      std::vector<double>& v,
      double mean);

  template <typename ContainerT>
  double timeVarianceCalculator(
      const ContainerT& observations,
      std::vector<double>& timeWeights);
  
  vw::Vector3 Unit(vw::Vector3& v);                     
};

#include <boost/math/distributions/normal.hpp>

/* timeVarianceCalculator
 * const std::List<OrbitalReading>& readings 
 * -A pointer to the list of readings
 * 
 * std::vector<double>& timeWeights&
 * -a pointer for the weights of each time stamp
 *
 * Returns the average of the time variance

*/
template <typename ContainerT>
inline double WeightCalculator::timeVarianceCalculator(
    const ContainerT& readings,
    std::vector<double>& timeWeights)
{
  std::vector<double> tDelta;
  unsigned int i = 0;
  double tSum = 0;
  double tMean = 0;
  double tSigma;
  
  typename ContainerT::const_iterator it_prev = readings.begin();
  typename ContainerT::const_iterator it_next = it_prev;
  ++it_next;
  
  for ( ; it_next != readings.end(); ++it_prev, ++it_next)
  {
    tDelta.push_back(it_next->mTime - it_prev->mTime);
  }
  
  for(i = 0; i < tDelta.size(); i++)
  {
    tSum += tDelta[i]; 
  }
  
  tMean = tSum / tDelta.size();
  tSum = 0;
  
  tSigma = standardDeviationOneDvector(tDelta, tMean);
  
  if( tMean == 0 || tSigma == 0) //This is a perfect data set
  {
    for(i = 0; i < tDelta.size(); i++)
    { timeWeights.push_back(1); }
    tSum = timeWeights.size(); 
  }
  else
  {
      //Calculate the normal probability distribution
    boost::math::normal_distribution<double> nd(tMean, tSigma);
    for(i = 0; i< tDelta.size(); i++)
    {
        //Get the probability of each time reading
      timeWeights.push_back( boost::math::pdf(nd, tDelta[i]));
      tSum += timeWeights[i];
    }
  }
  return tSum / timeWeights.size();
}

/*
   Main interface for the WeightCalculator
   TODO Orbital Reading shoudl be a const, but I dont know how to 
   Iterate with it as const. Couldnt get it to compile
   */
template <typename ContainerT>
inline void WeightCalculator::calculateWeights(
    const ContainerT& observations,
    const std::vector<vw::Vector3>& estimated_locations,
    std::vector<double>& weights)
{
  //Calculate time variance
  //Create a vector of the Error vectors (R)
  vw::Vector3 eTmp; //Temporary container
  std::vector<double> errors; //vector of the error values
  vw::Vector3 errorVector; //Error Vector
  vw::Vector3 rUnit; //Radial component of the error. Temporary container
  unsigned int i;
  double tAvg;
  double meanOfWeights = 0;
  std::vector<double> timeWeights;
  
  i = 0;
  for (typename ContainerT::const_iterator it = observations.begin();
       it != observations.end(); ++it, ++i)
  {
    errorVector[0] = estimated_locations[i][0] - it->mCoord[0]; 
    errorVector[1] = estimated_locations[i][1] - it->mCoord[1]; 
    errorVector[2] = estimated_locations[i][2] - it->mCoord[2];
    
    vw::Vector3 reverse;
    reverse[0] = it->mCoord[0];
    reverse[1] = it->mCoord[1];
    reverse[2] = it->mCoord[2];
    
    //rUnit = WeightCalculator::Unit(it->mCoord);
    rUnit = WeightCalculator::Unit(reverse);
    
    //Find just the R component of the error. with the dot product
    double r_error = dot_prod(rUnit, errorVector);
    
    errors.push_back(r_error);
  }
  
  //Calculate the Time Variance
  tAvg = WeightCalculator::timeVarianceCalculator(observations, timeWeights);
  
  //Update the weights with Gradient Decent
  meanOfWeights = WeightCalculator::smart_weighted_mean(
      weights,
      timeWeights,
      tAvg,
      errors,
      0.01, //Sig Level         
      0.01, //Learning Rate
      0.01, //Error Tolerance
      100);//Iterations
  
  //Factor in the time
  return;
}


#endif
