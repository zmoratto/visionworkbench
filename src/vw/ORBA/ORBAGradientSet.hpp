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
#include <boost/foreach.hpp>

namespace vw {
namespace ORBA {

using namespace vw;
using namespace vw::math;

// Gradients values are:
//  * trajectory
//    * GM, p0, v0 (7 total)
//    * 1 per reading (the timestamp)
//  * x_k
//    * 3 vars per landmark (control point)
//    * 7 vars per reading (number of readings is the same as t.size())
//      * 3 for pj
//      * 4 for cj_second
//    * 9 vars for our precisions
// That gives us a total of 3 per control point, 8 per reading, + 16
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
    x_k.set_size(vars.cnet->size()*3 + t.size()*7 + 9);
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
    
inline ORBAGradientSet operator*(double lhs, const ORBAGradientSet& rhs)
{
  ORBAGradientSet result;
  result.trajectory = lhs*rhs.trajectory;
  result.x_k = lhs*rhs.x_k;
  return result;
}

inline ORBAGradientSet& operator+=(ORBAGradientSet& lhs,
                                   const ORBAGradientSet& rhs)
{
  lhs.trajectory += rhs.trajectory;
  lhs.x_k += rhs.x_k;
  return lhs;
}

inline ORBADecisionVariableSet operator+(const ORBADecisionVariableSet& lhs,
                                 const ORBAGradientSet& rhs)
{
  using namespace vw::ba;
  
  ORBADecisionVariableSet result(lhs);
  result.trajectory = lhs.trajectory + rhs.trajectory;
    // The rest of the gradients are found in x_k.
    // We'll keep a running index
  std::size_t i_x_k = 0;

    // Three gradients per control point in the control network
  BOOST_FOREACH(ControlPoint& cp, *result.cnet)
  {
    Vector3 b = cp.position();
    b += subvector(rhs.x_k, i_x_k, 3);
    cp.set_position(b);
    i_x_k += 3;
  }

    // three gradients for each reading for pj
  BOOST_FOREACH(Vector3& pj, result.pj)
  {
    pj += subvector(rhs.x_k, i_x_k, 3);
    i_x_k += 3;
  }
  
    // Four gradients for each reading for cj_second
  BOOST_FOREACH(Vector4& cj, result.cj_second)
  {
    cj += subvector(rhs.x_k, i_x_k, 4);
    i_x_k += 4;
  }

  result.precision_p += subvector(rhs.x_k, i_x_k, 2);
  i_x_k += 2;
  result.precision_r += subvector(rhs.x_k, i_x_k, 3);
  i_x_k += 3;
  result.precision_s += subvector(rhs.x_k, i_x_k, 3);
  i_x_k += 3;
  result.precision_t += rhs.x_k[i_x_k];
  
  return result;
}

}} // vw::ORBA


#endif	/* ORBA_GRADIENT_SET_HPP */

