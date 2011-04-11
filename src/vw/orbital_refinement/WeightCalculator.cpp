#include <iostream>
#include <vw/orbital_refinement/WeightCalculator.hpp>
#include <vw/orbital_refinement/TrajectoryGradientSet.hpp>
#include <vw/Core/FundamentalTypes.h>

#include <boost/program_options.hpp>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Cartography.h>
#include <boost/math/distributions/fisher_f.hpp>

namespace po = boost::program_options;
using namespace vw;


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


// OUTPUT: weights are updated 
// RETURN: mean value
double WeightCalculator::smart_weighted_mean(
    std::vector<double>& weights, // initial and final weights in [0,1] corresponding to samples
    const std::vector<double>& tWeights, // input weights of the time
    double tAvg, //average of the time variance
    const std::vector<double> & samples,// sample data
    const double sign_level, // significance level for statistical testing
    const double learn_rate, // learning rate of gradient decent method
    const double error_tol, // error tolerance
    const long max_iter// maximum number of iteration
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
            //double sse_wt = 0; // squared sum of weight differences
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

            //Integrate Time Variance
            p_value = p_value * tWeights[j] / tAvg;

            //Interpolate
            if(p_value < sign_level)
            {
               p_value = (-1) + (p_value / sign_level);
            }
            else 
            {
               p_value = ( p_value - sign_level) / (1 - sign_level);
            }
            
              // gradient decent update
            weights[j] += learn_rate*(p_value);
              // 0.0 is lower bound of weight
            if ( weights[j] < 0 ) weights[j] = 0;
              // 1.0 is upper bound of weight
            if ( weights[j] > 1 ) weights[j] = 1;
            
              // squared sum of weight differences
              //sse_wt += (weights[j]-prev_wt[j])*(weights[j]-prev_wt[j]);
          }
	
            // terminal condition: mean squared difference of weights is smaller than the tolerance
       //   if ( sse_wt/weights.size() < error_tol )
       //     break;
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
