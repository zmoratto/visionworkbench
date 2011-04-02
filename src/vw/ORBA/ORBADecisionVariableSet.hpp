/* 
 * File:   ORBADecisionVariableSet.hpp
 * Author: hfung
 *
 * Created on April 1, 2011, 11:31 PM
 */

#ifndef ORBA_DECISION_VARIABLE_SET_HPP
#define	ORBA_DECISION_VARIABLE_SET_HPP

#include <vw/orbital_refinement/TrajectoryDecisionVariableSet.hpp>

using namespace vw::math;
using namespace vw::camera;
using namespace vw::ba;

struct ORBADecisionVariableSet : public TrajectoryDecisionVariableSet {
 // The new parameters
 ControlNetwork& cnet; // We'll be modifying the Control Network in-place
 std::vector<Vector3> pj; // a.k.a the first half of Cj
 std::vector<Vector4> cj_second; // the second half of cj. These are quaternions but
                                 // we are not using the object directly. Why? Because
                                 // quaternions need to be normalized, and that's a hard
                                 // constraint to enforce in this object when we are taking
                                 // gradients. The normalization will happen when we
                                 // construct the wrapper Cj around the camera models for
                                 // evaluation of Ep (projection).
 // Instead of using std deviations / sigmas like Taemin's
 // equations, it is more efficient computer-wise to be solving
 // for the precision (aka inverse variance).
 Vector3 precision_p; // reprojection, seed with [1,1]
 Vector3 precision_r; // between p and s, seed with [1,1,1]
 Vector3 precision_s; // between q and s, seed with camera sigma
 double precision_t;  // seed with sigma t

 ORBADecisionVariableSet( double GM_in, const& Vector3 p0_in,
                          const& Vector3 v0_in, std::list<OrbitalCameraReading> const& obs,
                          Vector3 const& sigma_p, Vector3 const& sigma r,
                          Vector3 const& sigma_s, double sigma_t ) :
   TrajectoryDecisionVariableSet( GM_in, p0_in, v0_in, obs ) {
  // User passes in sigmas instead of precision as that is more intuitive for
  // the user to understand
  precision_p = 1/elem_prod(sigma_p,sigma_p);
  precision_r = 1/elem_prod(sigma_r,sigma_r);
  precision_s = 1/elem_prod(sigma_s,sigma_s);
  precision_t = 1/( sigma_t * sigma_t );
 }
};

#endif	/* ORBA_DECISION_VARIABLE_SET_HPP */

