#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Cartography.h>
using namespace vw;

#include <boost/math/distributions/fisher_f.hpp>

public float WeightCalculator::smart_weighted_mean(
													 std::vector<float>				& weights,				// initial and final weights in [0,1] corresponding to samples
													 std::vector<float> const & samples,				// sample data
													 float const								sign_level,			// significance level for statistical testing
													 float const								learn_rate,			// learning rate of gradient decent method
													 float const								error_tol,			// error tolerance
													 long const									max_iter)				// maximum number of iteration
// OUTPUT: weights are updated 
// RETURN: mean value
{
	using namespace boost::math;
	
	switch ( samples.size() ) {
		case 1:
			weights[0]=1;
			return samples[0];
		case 2:
			weights[0]=1;
			weights[1]=1;
			return (samples[0]+samples[1])/2;
		default:
			
			float weighted_mean;										// weighted mean
			std::vector<float> prev_wt = weights;		// the previous weights
			for (int i = 0; i < max_iter; ++i) {
				
				// accumulation of all sums
				float SW1 = 0;	// sum of weights
				float SW2 = 0;	// sum of squared weights
				float SWX = 0;	// weighted sum of samples
				float WX2 = 0;	// weighted sum of squared data
				for (size_t j = 0; j < samples.size(); ++j) {
					SW1 += weights[j];
					SW2 += weights[j]*weights[j];
					SWX += weights[j]*samples[j];
					WX2 += weights[j]*samples[j]*samples[j];
				}
				
				// weighted mean
				weighted_mean = SWX/SW1;
				
				float DN = SW1*WX2-SWX*SWX;				// denominator
				float v2 = SW1-SW2/SW1;						// the second degree of freedom
				float sse_wt = 0;									// squared sum of weight differences
				for (size_t j = 0; j < samples.size(); ++j) {
					
					// F statistic
					float DF = samples[j]-weighted_mean;	// difference from the mean
					float FS = (SW1*SW1-SW2)*DF*DF/DN;		// F statistic
					
					// degree of freedom
					float v1 = 1-weights[j]/SW1;				// the first degree of freedom
					
					// p-value calculation
					float p_value = 1;									// p-value 
					if (SW1 > 1 && DN > 0) {						// basic assumptions
						fisher_f dist(v1,v2);
						p_value = 1-cdf(dist, FS);
					}
					
					// gradient decent update
					weights[j] += learn_rate*(p_value-sign_level);
					if ( weights[j] < 0 ) weights[j] = 0;		// 0.0 is lower bound of weight
					if ( weights[j] > 1 ) weights[j] = 1;		// 1.0 is upper bound of weight
					
					// squared sum of weight differences
					sse_wt += (weights[j]-prev_wt[j])*(weights[j]-prev_wt[j]);
					prev_wt[j] = weights[j];		// update previous weight
					
				}
				
				// terminal condition: mean squared difference of weights is smaller than the tolerance
				if ( sse_wt/weights.size() < error_tol ) break;
			}
			
			return  weighted_mean;
	}
}

// Standard interface
/*
int main( int argc, char *argv[] ) {
	
	std::vector<float> weights;
	std::vector<float> samples;

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
	
	float robust_mean = smart_weighted_mean(weights, samples,	0.3, 0.1, 1e-5, 1000);
	std::cout << "Robust mean: " << robust_mean << std::endl; 

	std::cout << "Final weights: "; 
  for ( size_t i = 0; i < weights.size(); ++i ) std::cout << weights[i] << " "; 
	std::cout << std::endl;

}*/
