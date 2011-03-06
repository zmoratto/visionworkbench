#include <iostream>
#include <vw/orbital_refinement/WeightCalculator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <vw/orbital_refinement/TrajectoryGradientSet.hpp>
#include <vw/Core/FundamentalTypes.h>

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

double WeightCalculator::norm(vw::Vector3& v)
{
   double x,y,z;
   x = v[0]*v[0];
   y = v[1]*v[1];
   z = v[2]*v[2];
   return sqrt(x + y + z);
}


//Main interface for the WeightCalculator
//TODO Orbital Reading shoudl be a const, but I dont know how to 
//Iterate with it as const
void WeightCalculator::calculateWeights(std::list<OrbitalReading>& observations,
                           const std::vector<vw::Vector3>& estimated_locations,
                            std::vector<double>& weights)
{
 
   //Calculate time variance
   //Create a vector of the Error vectors (R)
   //std::vector<vw::Vector3> eTmp;
   vw::Vector3 eTmp;
   std::vector<double> errors;
   std::vector<vw::Vector3> errorVector;
   vw::Vector3 rUnit;
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
      /*
      x2 = (eTmp->mCoord[0]) * (eTmp->mCoord[0]);
      y2 = (eTmp->mCoord[1]) * (eTmp->mCoord[1]);
      z2 = (eTmp->mCoord[2]) * (eTmp->mCoord[2]);
      */
   
      //normalError = sqrt(x2 + y2 + z2);
      //normalError = vw::Vector3::norm_1(eTmp);
      rUnit = WeightCalculator::norm(it->mCoord);

      //Find just the R component of the error. 
      //TODO Link the dot_prod function in
      //errors.push_back( dot_prod(&normalError, it->mCoord ));
/*
      x1 = rUnit[0];
      y1 = rUnit[1];
      z1 = rUnit[2];
      x2 = it->mCoord[0];
      y2 = it->mCoord[1];
      z2 = it->mCoord[2];
*/
      x = rUnit[0] * it->mCoord[0];
      y = rUnit[1] * it->mCoord[1];
      z = rUnit[2] * it->mCoord[2];
      
/*
      x = x1 *x2;
      y = y1 *y2;
      z = z1 *z2;
*/
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



