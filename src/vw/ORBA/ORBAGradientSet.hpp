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
#include <vw/ORBA/ORBADecisionVariableSet.hpp>

namespace vw {
namespace ORBA {

using namespace vw;
using namespace vw::math;

struct ORBAGradientSet
{
    // trajectory-related gradients
  TrajectoryGradientSet trajectory;
  
    // Instead of copying CNET, just have a big vector of values.
    // Consider whether we want to put more structure into this.
  Vector<float64> x_k;

    // Aliases for convenience, preferred over inheritence in this case.
  double& GM;
  Vector3& p0;
  Vector3& v0;
  std::vector<double>& t;

    // Default constructor
  ORBAGradientSet() 
          : GM(trajectory.GM),
            p0(trajectory.p0),
            v0(trajectory.v0),
            t(trajectory.t)
      {}

    // Constructor that allocates space based on the sizes
    // in a decision variable set.
  ORBAGradientSet(const ORBADecisionVariableSet& vars)
          : GM(trajectory.GM),
            p0(trajectory.p0),
            v0(trajectory.v0),
            t(trajectory.t)
  {
      // Make arrays the same size as our input
    t.resize(vars.timestamps.size());
      // In addition to our trajectory gradients, there are:
      //  * 3 vars per landmark (control point)
      //  * 7 vars per reading (number of readings is the same as t.size())
      //    * 3 for pj
      //    * 4 for cj_second
      //  * 9 vars for our precisions
    x_k.set_size(vars.cnet.size()*3 + t.size()*7 + 9);
  }
  
  
  ORBAGradientSet& operator=( const ORBAGradientSet& rhs )
  {
    trajectory = rhs.trajectory;
    x_k = rhs.x_k;
    return *this;
  }
  
  ORBAGradientSet operator-()
  {
    ORBAGradientSet result;
    result.trajectory = -trajectory;
    result.x_k = -x_k;
    return result;
  }
};

inline double dot_prod(const ORBAGradientSet& lhs,
                       const ORBAGradientSet& rhs )
{
  double result = dot_prod(lhs.trajectory, rhs.trajectory);
  result += dot_prod(lhs.x_k, rhs.x_k);
  return result;
}
    
}} // vw::ORBA


#endif	/* ORBA_GRADIENT_SET_HPP */

