
// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#ifndef __VW_BUNDLEADJUSTMENT_BA_CG_MODEL_H
#define	__VW_BUNDLEADJUSTMENT_BA_CG_MODEL_H


#include <vw/Math/Vector.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Math/LinearAlgebra.h>
#include <vw/Math/MatrixSparseSkyline.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/BundleAdjustment.h>

using namespace vw::camera;

namespace vw{
namespace ba{

class BACGModel : public ba::ModelBase< BACGModel, 6, 3 > {
  typedef Vector<double, 6> camera_vector_t;
  typedef Vector<double, 3> point_vector_t;

  std::vector< boost::shared_ptr<PinholeModel> > m_cameras;
  boost::shared_ptr<ControlNetwork> m_cnet;
  std::vector<camera_vector_t> a, a_target;
  std::vector<point_vector_t> b, b_target;
  size_t m_num_pixel_observations;

public:
  // Constructor
    BACGModel( std::vector< boost::shared_ptr<PinholeModel> > const& cameras,
               boost::shared_ptr<ControlNetwork> network,
               Vector<double, 3> or_data ) : m_cameras(cameras), m_cnet(network) {
        // Compute the number of observations from the bundle.
        m_num_pixel_observations = 0;
        for (size_t i = 0; i < network->size(); ++i)
          m_num_pixel_observations += (*network)[i].size();

        // Setting up the A vectors
        a.resize( m_cameras.size() );
        a_target.resize( a.size() );
        for ( size_t j = 0; j < m_cameras.size(); j++ ) {
          a[j] = camera_vector_t();
          // Store the orbital refinement data to the first 3 positions of a
          for ( int i = 0; i < 3; i++ ) {
              a[j][i] = or_data[i];
          }
          a_target[j] = a[j];
        }

        // Setting up the B vectors
        b.resize( m_cnet->size() );
        b_target.resize( b.size() );
        for ( size_t i = 0; i < m_cnet->size(); i++ ) {
          b[i] = (*m_cnet)[i].position();
          b_target[i] = b[i];
        }
    }
   // -- REQUIRED STUFF ---------------------------------------

  // Access to the cameras
  Vector2 operator() ( size_t /*i*/, size_t j,
                       camera_vector_t const& a_j,
                       point_vector_t const& b_i ) const {
    // Quaternions are the last half of this equation
    AdjustedCameraModel cam( m_cameras[j],
                             subvector(a_j,0,3),
                             math::euler_to_quaternion(a_j[3],a_j[4],a_j[5],"xyz") );

    return cam.point_to_pixel( b_i );
  }

  inline Matrix<double,6,6> A_inverse_covariance( size_t /*j*/ ) {
    Matrix<double,6,6> result;
    result.set_identity();
    result *= 2;
    return result;
  }
  inline Matrix<double,3,3> B_inverse_covariance( size_t /*i*/ ) {
    Matrix<double,3,3> result;
    result.set_identity();
    result *= 2;
    return result;
  }

  size_t num_cameras() const { return a.size(); }
  size_t num_points() const { return b.size(); }
  camera_vector_t A_parameters( size_t j ) const { return a[j]; }
  point_vector_t B_parameters( size_t i ) const { return b[i]; }
  camera_vector_t A_target( size_t j ) const { return a_target[j]; }
  point_vector_t B_target( size_t i ) const { return b_target[i]; }
  size_t num_pixel_observations() const { return m_num_pixel_observations; }
  void set_A_parameters(size_t j, camera_vector_t const& a_j) { a[j] = a_j; }
  void set_B_parameters(size_t i, point_vector_t const& b_i) { b[i] = b_i; }

  boost::shared_ptr<ControlNetwork> control_network(void) {
    return m_cnet; }
};

}
}

#endif	/* __VW_BUNDLEADJUSTMENT_BA_CG_MODEL_H */

