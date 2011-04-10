/* 
 * File:   ORBADecisionVariableSet.hpp
 * Author: hfung
 *
 * Created on April 1, 2011, 11:31 PM
 */

#ifndef ORBA_DECISION_VARIABLE_SET_HPP
#define ORBA_DECISION_VARIABLE_SET_HPP

#include <vw/orbital_refinement/TrajectoryDecisionVariableSet.hpp>
#include <vw/ORBA/OrbitalCameraReading.hpp>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vector>

namespace vw {
namespace ORBA {

using namespace vw;
using namespace vw::ba;

// Our decision variables are:
//   * Trajectory variables, namely:
//     * GM, p0, v0
//     * One timestamp per reading.
//   * For each ControlPoint (landmark):
//     * landmark coordinates (3 doubles)
//     Note that the landmark coordinates are stored right in
//     the control network instead of having a separate vector.
//   * For each camera, the following:
//     * camera coordinates (3 doubles)
//     * Orientation values (4 doubles)
//   * A set of precision values.  These get modified by CG just
//     like all the other decision variables.
struct ORBADecisionVariableSet
{
    // Satellite trajectory decision variables
  TrajectoryDecisionVariableSet trajectory;

    // ControlPoint variables (landmarks).
    // The piece of the cnet that is considered part of the
    // decision variable vector are the ControlPoint coordinates,
    // which are the calculated 3D coordinates of each landmark.
    // We'll be modifying them in-place in the Control Network.
  boost::shared_ptr<ControlNetwork> cnet;
  
    // Camera coordinates; the first half of Cj.
  std::vector<Vector3> pj;

    // Camera orientation; the second half of cj.
    // These are quaternions but
    // we are not using the object directly. Why? Because
    // quaternions need to be normalized, and that's a hard
    // constraint to enforce in this object when we are taking
    // gradients. The normalization will happen when we
    // construct the wrapper Cj around the camera models for
    // evaluation of Ep (projection).
  std::vector<Vector4> cj_second;
  
    // Instead of using std deviations / sigmas like Taemin's
    // equations, it is more efficient computer-wise to be solving
    // for the precision (aka inverse variance).
  Vector2 precision_p; // reprojection, seed with [1,1]
  Vector3 precision_r; // between p and s, seed with [1,1,1]
  Vector3 precision_s; // between q and s, seed with camera sigma
  double precision_t;  // seed with sigma t

    // Aliases for convenience, preferred over inheritence in this case.
  double& GM;
  Vector3& p0;
  Vector3& v0;
  std::vector<OrbitalReading::timestamp_t>& timestamps;

    // Fully populated constructor
  template <typename CollectionT>
  ORBADecisionVariableSet( double GM_in, const Vector3& p0_in,
                           const Vector3& v0_in,
                           const CollectionT& obs,
                           boost::shared_ptr<ControlNetwork> control_net,
                           const Vector2& sigma_p,
                           const Vector3& sigma_r,
                           const Vector3& sigma_s, double sigma_t ) :
      trajectory( GM_in, p0_in, v0_in, obs ),
      cnet(control_net),
      GM(trajectory.GM),
      p0(trajectory.p0),
      v0(trajectory.v0),
      timestamps(trajectory.timestamps)
  {
      // User passes in sigmas instead of precision as that is
      // more intuitive for the user to understand
    precision_p = elem_quot(1, elem_prod(sigma_p,sigma_p));
    precision_r = elem_quot(1, elem_prod(sigma_r,sigma_r));
    precision_s = elem_quot(1, elem_prod(sigma_s,sigma_s));
    precision_t = 1/( sigma_t * sigma_t );
  }

    // Copy constructor
  ORBADecisionVariableSet(const ORBADecisionVariableSet& to_copy)
          : trajectory(),
            cnet(to_copy.cnet),
            precision_p(to_copy.precision_p),
            precision_r(to_copy.precision_r),
            precision_s(to_copy.precision_s),
            precision_t(to_copy.precision_t),
            GM(trajectory.GM),
            p0(trajectory.p0),
            v0(trajectory.v0),
            timestamps(trajectory.timestamps)
  {
    GM = to_copy.GM;
    p0 = to_copy.p0;
    v0 = to_copy.v0;
    timestamps = to_copy.timestamps;
  }

    // default constructor
  ORBADecisionVariableSet()
          : trajectory(),
            GM(trajectory.GM),
            p0(trajectory.p0),
            v0(trajectory.v0),
            timestamps(trajectory.timestamps)
  {
        
  }

  ORBADecisionVariableSet& operator=(const ORBADecisionVariableSet& rhs)
  {
    trajectory = rhs.trajectory;
    cnet = rhs.cnet;
    pj = rhs.pj;
    cj_second = rhs.cj_second;
    precision_p = rhs.precision_p;
    precision_r = rhs.precision_r;
    precision_s = rhs.precision_s;
    precision_t = rhs.precision_t;
    return *this;
  }

};

}} // namespace vw::ORBA


#endif	/* ORBA_DECISION_VARIABLE_SET_HPP */

