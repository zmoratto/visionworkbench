/* 
 * File:   ORBAGradientSet.hpp
 * Author: hfung
 *
 * Created on April 2, 2011, 2:47 PM
 */

#ifndef ORBA_GRADIENT_SET_HPP
#define	ORBA_GRADIENT_SET_HPP

#include <vw/orbital_refinement/TrajectoryGradientSet.hpp>
#include <vw/Math/Vector.h>

namespace vw {
namespace ORBA {

using namespace vw;
using namespace vw::math;

struct ORBAGradientSet
{
    // trajectory-related gradients
  TrajectoryGradientSet trajectory_gradients;
  
    // Instead of copying CNET, just have a:
    // consider how we want to structure this.
  Vector<float64> x_k;
  
  ORBAGradientSet& operator=( const ORBAGradientSet& rhs )
  {
    trajectory_gradients = rhs.trajectory_gradients;
    x_k = rhs.x_k;
    return *this;
  }
  
  ORBAGradientSet operator-()
  {
    ORBAGradientSet result;
    result.trajectory_gradients = -trajectory_gradients;
    result.x_k = -x_k;
    return result;
  }
};

inline double dot_prod(const ORBAGradientSet& lhs,
                       const ORBAGradientSet& rhs )
{
  double result = dot_prod(lhs.trajectory_gradients, rhs.trajectory_gradients);
  result += dot_prod(lhs.x_k, rhs.x_k);
  return result;
}
    
}} // vw::ORBA


#endif	/* ORBA_GRADIENT_SET_HPP */

