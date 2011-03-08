#include <iostream>
#include <vw/orbital_refinement/WeightCalculator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <vw/orbital_refinement/TrajectoryGradientSet.hpp>
#include <vw/Core/FundamentalTypes.h>


/*
   This was faster than finding the right function to do this
   and figuring out how to use it. Just a quick and dirty 
   Standard deviation calculator
*/
double WeightCalculator::standardDeviationOneDvector(
                           std::vector<double>& v,
                           double mean)
{
   int i = 0;
   double sz = v.size();
   double sum = 0;

   for(i = 0; i < sz; i++)
   {
      sum += (v[i] - mean) * (v[i] - mean);
   }
   sum /= sz;

   return sqrt(sum);
}


/* timeVarianceCalculator
 * const std::List<OrbitalReading>& readings 
 * -A pointer to the list of readings
 * 
 * std::vector<double>& timeWeights&
 * -a pointer for the weights of each time stamp
*/
void WeightCalculator::timeVarianceCalculator(
                        const std::list<OrbitalReading>& readings,
                        std::vector<double>& timeWeights)
{
    std::vector<double> tDelta;
    unsigned int i = 0;
    double tSum = 0;
    double tMean;
    double tSigma;

    std::list<OrbitalReading>::const_iterator it_prev = readings.begin();
    std::list<OrbitalReading>::const_iterator it_next = it_prev;
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

    tSigma = standardDeviationOneDvector(tDelta, tMean);
  
    //TODO Finish this calculation, what does this mean?
    boost::normal_distribution<double> nd(tMean, tSigma);

    return;
}

/* WeightCalculator::Unit(vw::Vector3& v)
 * 
 * This utility function returns the unit of a vector
 * This probably exists somewhere else, but it was easier and faster to just
 * write this.
 */
vw::Vector3 WeightCalculator::Unit(vw::Vector3& v)
{
   double x,y,z, mag;
   vw::Vector3 unit;
   x = v[0]*v[0];
   y = v[1]*v[1];
   z = v[2]*v[2];
   mag = sqrt(x + y + z);
   unit[0] = v[0] / mag;
   unit[1] = v[1] / mag;
   unit[2] = v[2] / mag;
   return unit;
}


/*
   Main interface for the WeightCalculator
   TODO Orbital Reading shoudl be a const, but I dont know how to 
   Iterate with it as const. Couldnt get it to compile
   */
void WeightCalculator::calculateWeights(std::list<OrbitalReading>& observations,
                           const std::vector<vw::Vector3>& estimated_locations,
                            std::vector<double>& weights)
{
 
   //Calculate time variance
   //Create a vector of the Error vectors (R)
   vw::Vector3 eTmp; //Temporary container
   std::vector<double> errors; //vector of the error values
   std::vector<vw::Vector3> errorVector; //Vector of Error Vectors
   vw::Vector3 rUnit; //Radial component of the error. Temporary container
   unsigned int i;
   double x, y, z;
   float meanOfWeights = 0;

   i = 0;
   for (std::list<OrbitalReading>::iterator it = observations.begin();
            it != observations.end(); ++it, ++i)
   {
     errorVector[0] = estimated_locations[i][0] - it->mCoord[0]; 
     errorVector[1] = estimated_locations[i][1] - it->mCoord[1]; 
     errorVector[2] = estimated_locations[i][2] - it->mCoord[2]; 
   
      rUnit = WeightCalculator::Unit(it->mCoord);

      //Find just the R component of the error. with the dot product
      x = rUnit[0] * it->mCoord[0];
      y = rUnit[1] * it->mCoord[1];
      z = rUnit[2] * it->mCoord[2];
      
      errors.push_back(x+y+z);
   }

   //Update the weights with Gradient Decent
   meanOfWeights = smart_weighted_mean(weights,
                        errors,
                        0.5, //Test Value, get good ones later
                        0.5, //Test Value
                        0.01, //Test Value
                        10);//Test Value

   //Factor in the time
   return;
}



