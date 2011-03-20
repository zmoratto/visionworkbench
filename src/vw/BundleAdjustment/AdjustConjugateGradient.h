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
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Math/ConjugateGradient.h>

namespace vw {
namespace ba {

  template <class BundleAdjustModelT>
  class BACGErrorCalculator
  {
  public:
    static const size_t PARAMS_PER_POINT = BundleAdjustModelT::point_params_n;
    static const size_t PARAMS_PER_CAMERA = BundleAdjustModelT::camera_params_n;
    const size_t DIMENSION;
    
    typedef double result_type;
      // We store our decision variables as one big array of doubles.
      // It's 6 doubles per camera followed by 3 doubles per ControlPoint.
    typedef Vector<float64> domain_type;
      // Gradient follows the same structure as the domain.
    typedef Vector<float64> gradient_type;

    typedef Vector<float64, PARAMS_PER_POINT> point_vector_t;
    typedef Vector<float64, PARAMS_PER_CAMERA> camera_vector_t;
    

    BACGErrorCalculator(BundleAdjustModelT& model) :
      DIMENSION(model.num_cameras()*PARAMS_PER_CAMERA +
                model.num_points() *PARAMS_PER_POINT),
      m_model(model)
    {}

    // Evaluate the error for the given set of decision variables.
    result_type operator()(const domain_type& x) const
    {
      const size_t points_domain_offset =
        BundleAdjustModelT::camera_params_n * m_model.num_cameras();
      const size_t camera_domain_offset = 0;
      
        // Iterate through the control network
      boost::shared_ptr<ControlNetwork> net = m_model.control_network();
      
        // Loop through each point in the model
      double error = 0;
      int i = 0;
      BOOST_FOREACH(const ControlPoint& cp, *net)
      {
          // Get the estimated location of this control point
        point_vector_t b = subvector(
          x,
          points_domain_offset + i*PARAMS_PER_POINT,
          PARAMS_PER_POINT);
        
          // Loop through each measure in the control point
        BOOST_FOREACH(const ControlMeasure& cm, cp)
        {
            // cm.position() is the pixel location of cp in this image.
            // It never changes.  We compare this actual pixel location
            // against the expected pixel location of real-world coordinate b.
          std::size_t this_camera_offset =
            camera_domain_offset + cm.image_id()*PARAMS_PER_CAMERA;
          camera_vector_t a = subvector(
            x, this_camera_offset, PARAMS_PER_CAMERA);
          
          error += norm_2(cm.position() - m_model(i, cm.image_id(), a, b));
        }
        i++;
      }
      return error;
    }

      // Evaluate the gradient for the given set of decision variables.
    gradient_type gradient(domain_type const& x) const
    {
        // hard-coded minimum delta
      const double epsilon = .01;
        
        // Create a copy of x.
      domain_type x_copy = x;

        // Create a place to store the gradients
      gradient_type gradient;
      gradient.set_size(x_copy.size());
      
        // Loop through each element of x_copy,
        // calculate a numeric gradient
      for (std::size_t i = 0; i < x_copy.size(); ++i)
      {
        double save = x_copy[i];
        double forward_x, reverse_x;
        
          // First do the forward difference
        if (fabs(save) < 1.0)
          forward_x = save + epsilon;
        else
          forward_x = save * 1.001;
        x_copy[i] = forward_x;
        double error = operator()(x_copy);
        
          // Now do reverse difference
        if (fabs(save) < 1.0)
          reverse_x = save - epsilon;
        else
          reverse_x = save * 0.999;
        x_copy[i] = reverse_x;
        error -= operator()(x_copy);

          // Restore the original value
        x_copy = save;

          // Save the gradient for this element
        gradient[i] = error/(forward_x - reverse_x);

      }
        // return the full result
      return gradient;
    }
    

      // the dimension of the gradient vector.
    unsigned dimension() const
    { return DIMENSION; }
    
    
  private:

      // Note that it's not const...not because we change it, but because
      // control_network() is non-const (it probably should be).
    BundleAdjustModelT& m_model;
  };
  
  template <class BundleAdjustModelT, class RobustCostT>
  class AdjustConjugateGradient :
    public AdjustBase<BundleAdjustModelT,RobustCostT>,
    private boost::noncopyable
  {
  public:
    
    AdjustConjugateGradient( BundleAdjustModelT& model,
                             RobustCostT const& robust_cost_func,
                             bool use_camera_constraint=true,
                             bool use_gcp_constraint=true ) :
      AdjustBase<BundleAdjustModelT,RobustCostT>( model, robust_cost_func,
                                                  use_camera_constraint,
                                                  use_gcp_constraint )
    {}
      
    double update(double &abs_tol, double &rel_tol)
    {
      const int MAX_CG_ITERATIONS = 500;

        // Create an error calculator
      BACGErrorCalculator<BundleAdjustModelT> err_calc(this->m_model);

        // Create an initial guess
      Vector<float64> x;
      initialize_domain_guess(x);

        // Get the error before CG
      double err_before = err_calc(x);

        // Run CG
      x = math::conjugate_gradient(err_calc, x, math::ArmijoStepSize(), MAX_CG_ITERATIONS);

        // Store the results back in the model
      store_domain_in_model(x);

        // What do I return...the change in error?
      return err_before - err_calc(x);
    }

  private:
    void initialize_domain_guess(Vector<float64>& x)
    {
        // Make it the right size
      x.set_size(this->m_model.num_cameras()*BundleAdjustModelT::camera_params_n +
                 this->m_model.num_points() *BundleAdjustModelT::point_params_n);

        // Populate it
      std::size_t index = 0;
      for (std::size_t i = 0; i < this->m_model.num_cameras(); ++i)
      {
         Vector<double,BundleAdjustModelT::camera_params_n> a =
           this->m_model.A_parameters(i);
         
         for (std::size_t j = 0; j < BundleAdjustModelT::camera_params_n; j++)
           x[index++] = a[j];
      }
      for (std::size_t i = 0; i < this->m_model.num_points(); ++i)
      {
         Vector<double,BundleAdjustModelT::point_params_n> b =
           this->m_model.B_parameters(i);
         
         for (std::size_t j = 0; j < BundleAdjustModelT::point_params_n; j++)
           x[index++] = b[j];
      }
    }

    void store_domain_in_model(const Vector<float64>& x)
    {
      std::size_t index = 0;
      for (std::size_t i = 0; i < this->m_model.num_cameras(); ++i)
      {
        this->m_model.set_A_parameters(
          i, subvector(x, index, BundleAdjustModelT::camera_params_n));
        index += BundleAdjustModelT::camera_params_n;
      }
      for (std::size_t i = 0; i < this->m_model.num_points(); ++i)
      {
        this->m_model.set_B_parameters(
          i, subvector(x, index, BundleAdjustModelT::point_params_n));
        index += BundleAdjustModelT::point_params_n;
      }
    }
  };
    
}}

#endif//__VW_BUNDLEADJUSTMENT_ADJUST_REF_H__
