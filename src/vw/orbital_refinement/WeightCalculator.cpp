#include <iostream>
#include "WeightCalculator.hpp"



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


void WeightCalculator::timeVarianceCalcuator(
                        const std::list<OrbitalReading>& observations,
                        std::vector<double>& timeWeights)
{
    std::vector<double> tDelta;
    int i = 0;
    double tSum = 0;
    double tMean;
    double tSigma;

    for (std::list<OrbitalReading>::iterator it = readings.begin();
         it != readings.end(); it++, i++)
    {
       if(it != readings.end())
       {
         tDelta.push_back((it+1)->mTime - it->mTime);
       }
    }

    for(i = 0; i < tDelta.size(); i++)
     {
        tSum += tDelta[i]; 
     }

     tMean = tSum / tDelta.size();

     tSigma = standardDeviationOneDvector(&tDelta, tMean);
   
      //tWeight = NormPDF(tDelta, tMean, tSigma) from boost lib

      

}





void WeightCalculator::calculateWeights(const std::list<OrbitalReading>& observations,
                           const std::vector<vw::Vector3>& estimated_locations,
                            std::vector<bool>& weights)

{
   
   return;
}



