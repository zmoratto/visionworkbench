#include <iostream>
#include <vw/orbital_refinement/WeightCalculator.hpp>


double WeightCalculator::standardDeviationOneDvector(
                           std::vector<double>& v,
                           double mean)
{
   int i = 0;
   double sz = v.size();
   double sum = 0;
   double sigma = 0;

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
    int i = 0;
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
   
      //tWeight = NormPDF(tDelta, tMean, tSigma) from boost lib

      

}





void WeightCalculator::calculateWeights(const std::list<OrbitalReading>& observations,
                           const std::vector<vw::Vector3>& estimated_locations,
                            std::vector<double>& weights)
{
 
   //Calculate time variance
   //Create a vector of the Error vectors (R)
   std::vector<vw::Vector3> eTmp;
   std::vector<double> errors;
   std::vector<vw::Vector3> normalError;
   int i;
   double x2, y2, z2;
   float meanOfWeights = 0;

   for(i = 0; i<observations.size();i++)
   {
      eTmp = estimated_locations[i] - observations[i];
      x2 = (eTmp->mCoord[0]) * (eTmp->mCoord[0]);
      y2 = (eTmp->mCoord[1]) * (eTmp->mCoord[1]);
      z2 = (eTmp->mCoord[2]) * (eTmp->mCoord[2]);
      normalError = sqrt(x2 + y2 + z2);
      //Find just the R component of the error. 
      //TODO Link the dot_prod function in
      errors.push_back( dot_prod(normalError, observations[i] );
   }
   //Call the robust mean function on it
   meanOfWeights = smart_weighted_mean(weights,
                        &errors,
                        .5, //Test Value, get good ones later
                        .5, //Test Value
                        10);//Test Value

   //Factor in the time
   return;
}



