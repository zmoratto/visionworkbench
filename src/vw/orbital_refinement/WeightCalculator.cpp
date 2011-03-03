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
   
   return;
}



