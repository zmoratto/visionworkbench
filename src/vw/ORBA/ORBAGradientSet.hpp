/* 
 * File:   ORBAGradientSet.hpp
 * Author: hfung
 *
 * Created on April 2, 2011, 2:47 PM
 */

#ifndef ORBA_GRADIENT_SET_HPP
#define	ORBA_GRADIENT_SET_HPP

#include <vw/orbital_refinement/TrajectoryGradientSet.hpp>

namespace vw {
namespace ORBA {

using namespace vw::math;

struct ORBAGradientSet
{
    // trajectory-related gradients
  TrajectoryGradientSet trajectory_gradients;
  
    // Instead of copying CNET, just have a:
  std::vector<Vector3> x_k;
  
  ORBAGradientSet& operator=( const ORBAGradientSet& rhs )
  {
      // I'm half sure this is correct
    (TrajectoryGradientSet)(*this) = (TrajectoryGradientSet const&)rhs;
      // Copy the new state parameters here
  }
  
  ORBAGradientSet operator-()
  {
      // Fresh implementation here as code reuse would cause an extra copy
  }
};

// I'm very confident that we can reuse math operations from
// TrajectoryGradientSet. Here's an example of one:
inline double dot_prod(const ORBAGradientSet& lhs,
                       const ORBAGradientSet& rhs )
{
 double result = dot_prod( (const TrajectoryGradientSet&)lhs,
                           (const TrajectoryGradientSet&)rhs );
 //result += // Rest of the new variables here.
 return result;
}
    
}} // vw::ORBA


#endif	/* ORBA_GRADIENT_SET_HPP */

