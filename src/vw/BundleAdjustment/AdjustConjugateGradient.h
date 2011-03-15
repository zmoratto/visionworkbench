// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file AdjustConjugateGradient.h
///
/// Conjugate Gradient implementation of bundle adjustment.

#ifndef __VW_BUNDLEADJUSTMENT_ADJUST_CONJUGATE_GRADIENT_H__
#define __VW_BUNDLEADJUSTMENT_ADJUST_CONJUGATE_GRADIENT_H__

#include <vw/BundleAdjustment/AdjustBase.h>
#include <vw/Math/LinearAlgebra.h>

// Boost
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/version.hpp>

// The sparse vectors/matrices are needed
// for the covariance calculation

#if BOOST_VERSION<=103200
// Mapped matrix doesn't exist in 1.32, but Sparse Matrix does
//
// Unfortunately some other tests say this doesn't work
#define boost_sparse_matrix boost::numeric::ublas::sparse_matrix
#define boost_sparse_vector boost::numeric::ublas::sparse_vector
#else
// Sparse Matrix was renamed Mapped Matrix in later editions
#define boost_sparse_matrix boost::numeric::ublas::mapped_matrix
#define boost_sparse_vector boost::numeric::ublas::mapped_vector
#endif

namespace vw {
namespace ba {

  template <class BundleAdjustModelT, class RobustCostT>
  class AdjustConjugateGradient : public AdjustBase<BundleAdjustModelT,RobustCostT>, private boost::noncopyable {

    // Need to save S for covariance calculations
    math::Matrix<double> m_S;

  public:

    AdjustConjugateGradient( BundleAdjustModelT & model,
               RobustCostT const& robust_cost_func,
               bool use_camera_constraint=true,
               bool use_gcp_constraint=true ) :
    AdjustBase<BundleAdjustModelT,RobustCostT>( model, robust_cost_func,
                                                use_camera_constraint,
                                                use_gcp_constraint ) {}

    Matrix<double> S() { return m_S; }
    void set_S(const math::Matrix<double>& S) {
      m_S = S;
    }

    // Covariance Calculator
    // __________________________________________________
    // This routine inverts a sparse matrix S, and prints the individual
    // covariance matrices for each camera
    void covCalc(){

      // camera params
      unsigned num_cam_params = BundleAdjustModelT::camera_params_n;
      unsigned num_cameras = this->m_model.num_cameras();

      unsigned inverse_size = num_cam_params * num_cameras;

      typedef Matrix<double, BundleAdjustModelT::camera_params_n, BundleAdjustModelT::camera_params_n> matrix_camera_camera;

      // final vector of camera covariance matrices
      vw::Vector< matrix_camera_camera > sparse_cov(num_cameras);

      // Get the S matrix from the model
      Matrix<double> S = this->S();
      Matrix<double> Id(inverse_size, inverse_size);
      Id.set_identity();
      Matrix<double> Cov = multi_solve_symmetric(S, Id);

      //pick out covariances of individual cameras
      for ( unsigned i = 0; i < num_cameras; i++ )
         sparse_cov(i) = submatrix(Cov, i*num_cam_params,
                                   i*num_cam_params,
                                   num_cam_params,
                                   num_cam_params);

      std::cout << "Covariance matrices for cameras are:"
                << sparse_cov << "\n\n";

      return;
    }

    // UPDATE IMPLEMENTATION
    //---------------------------------------------------------------
    // This is a simple, non-sparse, unoptimized implementation of LM
    // bundle adjustment.  It is primarily used for validation and
    // debugging.
    //
    // Each entry in the outer vector corresponds to a distinct 3D
    // point.  The inner vector contains a list of image IDs and
    // pixel coordinates where that point was imaged.
    double update(double &abs_tol, double &rel_tol) {
      
    }
  };

}}

#endif//__VW_BUNDLEADJUSTMENT_ADJUST_REF_H__
