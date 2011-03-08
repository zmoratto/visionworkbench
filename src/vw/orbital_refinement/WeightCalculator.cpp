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
   double meanOfWeights = 0;

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



#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Cartography.h>
using namespace vw;

#include <boost/math/distributions/fisher_f.hpp>

// OUTPUT: weights are updated 
// RETURN: mean value
double WeightCalculator::smart_weighted_mean(
    std::vector<double>& weights, // initial and final weights in [0,1] corresponding to samples
    std::vector<double> const & samples,// sample data
    const double sign_level, // significance level for statistical testing
    const double learn_rate, // learning rate of gradient decent method
    double const error_tol, // error tolerance
    long const max_iter// maximum number of iteration
    )
{
  using namespace boost::math;
  
  double weighted_mean = 0;
  switch ( samples.size() )
  {
    case 1:
        weights[0]=1;
        return samples[0];
    case 2:
        weights[0]=1;
        weights[1]=1;
        return (samples[0]+samples[1])/2;
    default:
        std::vector<double> prev_wt = weights; // the previous weights
        for (int i = 0; i < max_iter; ++i)
        {
            // accumulation of all sums
          double SW1 = 0;	// sum of weights
          double SW2 = 0;	// sum of squared weights
          double SWX = 0;	// weighted sum of samples
          double WX2 = 0;	// weighted sum of squared data
          for (size_t j = 0; j < samples.size(); ++j)
          {
            SW1 += weights[j];
            SW2 += weights[j]*weights[j];
            SWX += weights[j]*samples[j];
            WX2 += weights[j]*samples[j]*samples[j];
          }
	
            // weighted mean
          weighted_mean = SWX/SW1;
	
          double DN = SW1*WX2-SWX*SWX;// denominator
          double v2 = SW1-SW2/SW1;						// the second degree of freedom
          double sse_wt = 0; // squared sum of weight differences
          for (size_t j = 0; j < samples.size(); ++j)
          {
              // F statistic
            double DF = samples[j]-weighted_mean;	// difference from the mean
            double FS = (SW1*SW1-SW2)*DF*DF/DN;// F statistic
            
              // degree of freedom
            double v1 = 1-weights[j]/SW1; // the first degree of freedom
            
              // p-value calculation
            double p_value = 1; // p-value 
            if (SW1 > 1 && DN > 0)
            { // basic assumptions
              fisher_f dist(v1,v2);
              p_value = 1-cdf(dist, FS);
            }
            
              // gradient decent update
            weights[j] += learn_rate*(p_value-sign_level);
              // 0.0 is lower bound of weight
            if ( weights[j] < 0 ) weights[j] = 0;
              // 1.0 is upper bound of weight
            if ( weights[j] > 1 ) weights[j] = 1;
            
              // squared sum of weight differences
            sse_wt += (weights[j]-prev_wt[j])*(weights[j]-prev_wt[j]);
              // update previous weight
            prev_wt[j] = weights[j];
          }
	
            // terminal condition: mean squared difference of weights is smaller than the tolerance
          if ( sse_wt/weights.size() < error_tol )
            break;
        }
        
        return  weighted_mean;
  }
}

// Standard interface
/*
int main( int argc, char *argv[] ) {
	
	std::vector<double> weights;
	std::vector<double> samples;

	weights.push_back(0.5);
	weights.push_back(0.5);
	weights.push_back(0.5);
	weights.push_back(0.5);

	samples.push_back(0.1);
	samples.push_back(0.5);
	samples.push_back(0.7);
	samples.push_back(0.7);

	std::cout << "Initial weights: "; 
  for ( size_t i = 0; i < weights.size(); ++i ) std::cout << weights[i] << " "; 
	std::cout << std::endl;
	
	double robust_mean = smart_weighted_mean(weights, samples,	0.3, 0.1, 1e-5, 1000);
	std::cout << "Robust mean: " << robust_mean << std::endl; 

	std::cout << "Final weights: "; 
  for ( size_t i = 0; i < weights.size(); ++i ) std::cout << weights[i] << " "; 
	std::cout << std::endl;

}*/
