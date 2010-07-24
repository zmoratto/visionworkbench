// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// IntegralImage.h
//
// Provides support for algorithms that evaluate with Integral Images
// such as SURF
#ifndef __VW_INTERESTPOINT_INTEGRALIMAGE_H__
#define __VW_INTERESTPOINT_INTEGRALIMAGE_H__

#include <boost/utility/enable_if.hpp>
#include <vw/Core/FundamentalTypes.h>
#include <vw/Image/IntegralView.h>

namespace vw {
namespace ip {

  /// Integral Block Evaluation
  ///
  /// This is for summing an area of pixels described by integral
  template <class PixelT>
  inline PixelT
  IntegralBlock( ImageView<PixelT> const& integral,
                 Vector2i const& top_left,
                 Vector2i const& bottom_right ) {

#if defined(VW_ENABLE_BOUNDS_CHECK) && (VW_ENABLE_BOUNDS_CHECK==1)
    VW_ASSERT(top_left.x() < integral.cols(),
              vw::ArgumentErr() << "x0 out of bounds. "<< integral.cols() <<" : "
              << top_left << bottom_right << "\n");
    VW_ASSERT(bottom_right.x() < integral.cols(),
              vw::ArgumentErr() << "x1 out of bounds. "<< integral.cols() <<" : "
              << top_left << bottom_right << "\n");
    VW_ASSERT(top_left.y() < integral.rows(),
              vw::ArgumentErr() << "y0 out of bounds. "<< integral.rows() <<" : "
              << top_left << bottom_right << "\n");
    VW_ASSERT(bottom_right.y() < integral.rows(),
              vw::ArgumentErr() << "y1 out of bounds. "<< integral.rows() <<" : "
              << top_left << bottom_right << "\n");
#endif

    PixelT result;
    result = integral( top_left.x(), top_left.y() );
    result += integral( bottom_right.x(), bottom_right.y() );
    result -= integral( top_left.x(), bottom_right.y() );
    result -= integral( bottom_right.x(), top_left.y() );

    return result;
  }

  /// X First Derivative
  /// - x,y         = location to center the calculation on
  /// - filter_size = size of window for calculation
  template <class PixelT>
  inline PixelT
  XFirstDerivative( ImageView<PixelT> const& /*integral*/,
                    int const& /*x*/, int const& /*y*/,
                    unsigned const& /*filter_size*/ ) {
    vw_throw( vw::NoImplErr() << "First derivative filter has not been implemented yet\n" );
    PixelT derivative = 0;
    return derivative;
  }

  /// X Second Derivative
  /// - x,y         = location to center the calculation on
  /// - filter_size = size of window for calculation
  template <class PixelT>
  inline PixelT
  XSecondDerivative( ImageView<PixelT> const& integral,
                     int const& x, int const& y,
                     unsigned const& filter_size ) {
    unsigned lobe = filter_size / 3;
    unsigned half_lobe = (unsigned) floor( float(lobe) / 2.0 );
    PixelT derivative;

    // Adding positive left;
    derivative = IntegralBlock( integral,
                                Vector2i( x - lobe - half_lobe, y - lobe + 1 ),
                                Vector2i( x - half_lobe, y + lobe ) );

    // Adding negative middle;
    derivative -= 2.0*IntegralBlock( integral,
                                     Vector2i( x - half_lobe, y - lobe + 1 ),
                                     Vector2i( x + half_lobe + 1, y + lobe ) );

    // Adding positive right;
    derivative += IntegralBlock( integral,
                                 Vector2i( x + half_lobe + 1, y - lobe + 1 ),
                                 Vector2i( x + half_lobe + lobe + 1, y + lobe ) );

    derivative /= filter_size*filter_size;

    return derivative;
  }

  /// Y First Derivative
  /// - x,y         = location to center the calculation on
  /// - filter_size = size of window for calculation
  template <class PixelT>
  inline PixelT
  YFirstDerivative( ImageView<PixelT> const& /*integral*/,
                    int const& /*x*/, int const& /*y*/,
                    unsigned const& /*filter_size*/ ) {
    vw_throw( vw::NoImplErr() << "First derivative filter has not been implemented yet\n" );
    PixelT derivative = 0;
    return derivative;
  }

  /// Y Second Derivative
  /// - x,y         = location to center the calculation on
  /// - filter_size = size of window for calculation
  template <class PixelT>
  inline PixelT
  YSecondDerivative( ImageView<PixelT> const& integral,
                     int const& x, int const& y,
                     unsigned const& filter_size ) {
    unsigned lobe = filter_size / 3;
    unsigned half_lobe = (unsigned) floor( float(lobe) / 2.0 );
    PixelT derivative;

    // Adding positive top;
    derivative = IntegralBlock( integral,
                                Vector2i( x - lobe + 1, y - lobe - half_lobe ),
                                Vector2i( x + lobe, y - half_lobe ) );

    // Adding negative middle;
    derivative -= 2.0*IntegralBlock( integral,
                                     Vector2i( x - lobe + 1, y - half_lobe ),
                                     Vector2i( x + lobe, y + half_lobe + 1 ) );

    // Adding positive bottom;
    derivative += IntegralBlock( integral,
                                 Vector2i( x - lobe + 1, y + half_lobe + 1 ),
                                 Vector2i( x + lobe, y + half_lobe + lobe + 1 ) );

    derivative /= filter_size*filter_size;

    return derivative;
  }

  /// XY Derivative
  /// - x,y         = location to center the calculation on
  /// - filter_size = size of window for calculation
  template <class PixelT>
  inline PixelT
  XYDerivative( ImageView<PixelT> const& integral,
                int const& x, int const& y,
                unsigned const& filter_size ) {

    unsigned lobe = filter_size / 3;
    PixelT derivative;

    // Adding positive top left
    derivative = IntegralBlock( integral,
                                Vector2i( x - lobe, y - lobe ),
                                Vector2i( x, y ) );

    // Adding negative top right
    derivative -= IntegralBlock( integral,
                                 Vector2i( x + 1, y - lobe ),
                                 Vector2i( x + lobe + 1, y ) );

    // Adding negative bottom left
    derivative -= IntegralBlock( integral,
                                 Vector2i( x - lobe, y + 1 ),
                                 Vector2i( x, y + lobe + 1 ) );

    // Adding positve bottom right
    derivative += IntegralBlock( integral,
                                 Vector2i( x + 1, y + 1 ),
                                 Vector2i( x + 1 + lobe, y + 1 + lobe ) );

    derivative /= filter_size*filter_size;

    return derivative;
  }

  // Horizontal Wavelet
  // - integral  = Integral used for calculations
  // - x         = x location to evaluate at
  // - y         = y location to evaluate at
  // - size      = side of the square used for evaluate

  // Note: Filter will be evaluated at a size nearest to a multiple of two
  template <class ViewT>
  typename boost::disable_if<IsFloatingPointIndexable<ViewT>, float>::type
  inline HHaarWavelet( ImageViewBase<ViewT> const& integral,
                       int const& x, int const& y,
                       float const& size ) {

    float response;
    int half_size = int(round( size / 2.0));
    int i_size = half_size << 1;
    int top = int(round( y - size/2));
    int left = int(round( x - size/2));

#if defined(VW_ENABLE_BOUNDS_CHECK) && (VW_ENABLE_BOUNDS_CHECK==1)
    VW_ASSERT(left+i_size < integral.impl().cols(),
              vw::ArgumentErr() << "left out of bounds. "<< integral.impl().cols() <<" : "
              << left+i_size << " [top left] " << top << " " << left << "\n");
    VW_ASSERT(top+i_size < integral.impl().rows(),
              vw::ArgumentErr() << "top out of bounds. " << integral.impl().rows() <<" : "
              << top+i_size << "\n");
    VW_ASSERT(left >= 0,
              vw::ArgumentErr() << "left is too low. " << 0 << " : " << left << "\n");
    VW_ASSERT(top >= 0,
              vw::ArgumentErr() << "top is too low. " << 0 << " : " << top << "\n");
#endif

    response = -integral.impl()(left, top);
    response += 2*integral.impl()(left+half_size, top);
    response -= integral.impl()(left+i_size, top);
    response += integral.impl()(left, top+i_size);
    response -= 2*integral.impl()(left+half_size, top+i_size);
    response += integral.impl()(left+i_size, top+i_size);

    return response;
  }

  // Horizontal Wavelet ( floating point arithmetic )
  // - integral  = Integral used for calculations
  // - x         = x location to evaluate at
  // - y         = y location to evaluate at
  // - size      = side of the square used for evaluate

  // Note: This Filter requires/recommends the use of an interpolated
  //       view of the integral
  template <class ViewT>
  typename boost::enable_if<IsFloatingPointIndexable<ViewT>, float>::type
  inline HHaarWavelet( ImageViewBase<ViewT> const& integral,
                       float const& x, float const& y,
                       float const& size ) {

    float response;
    float half_size = size / 2.0;
    float top = y - half_size;
    float left = x - half_size;

    response = -integral.impl()(left, top);
    response += 2*integral.impl()(left+half_size, top);
    response -= integral.impl()(left+size, top);
    response += integral.impl()(left, top+size);
    response -= 2*integral.impl()(left+half_size, top+size);
    response += integral.impl()(left+size, top+size);

    return response;
  }

  // Vertical Wavelet
  // - integral  = Integral used for calculations
  // - x         = x location to evaluate at
  // - y         = y location to evaluate at
  // - size      = side of the square used for evaluate
  // Note: Filter will be evaluated at a size nearest to a multiple of two
  template <class ViewT>
  typename boost::disable_if<IsFloatingPointIndexable<ViewT>, float>::type
  inline VHaarWavelet( ImageViewBase<ViewT> const& integral,
                       int const& x, int const& y,
                       float const& size ) {

    float response;
    int half_size = int(round( size / 2.0));
    int i_size = half_size << 1;
    int top = int(round( y - size/2));
    int left = int(round( x - size/2));

#if defined(VW_ENABLE_BOUNDS_CHECK) && (VW_ENABLE_BOUNDS_CHECK==1)
    VW_ASSERT(left+i_size < integral.impl().cols(),
              vw::ArgumentErr() << "left out of bounds. "<< integral.impl().cols() <<" : "
              << left+i_size << " [top left] " << top << " " << left << "\n");
    VW_ASSERT(top+i_size < integral.impl().rows(),
              vw::ArgumentErr() << "top out of bounds. " << integral.impl().rows() <<" : "
              << top+i_size << "\n");
    VW_ASSERT(left >= 0,
              vw::ArgumentErr() << "left is too low. " << 0 << " : " << left << "\n");
    VW_ASSERT(top >= 0,
              vw::ArgumentErr() << "top is too low. " << 0 << " : " << top << "\n");
#endif

    response = -integral.impl()(left, top);
    response += integral.impl()(left+i_size, top);
    response += 2*integral.impl()(left, top+half_size);
    response -= 2*integral.impl()(left+i_size, top+half_size);
    response -= integral.impl()(left, top+i_size);
    response += integral.impl()(left+i_size, top+i_size);

    return response;
  }


  // Vertical Wavelet ( floating point arithmetic )
  // - integral  = Integral used for calculations
  // - x         = x location to evaluate at
  // - y         = y location to evaluate at
  // - size      = side of the square used for evaluate

  // Note: This Filter requires/recommends the use of an interpolated
  //       view of the integral
  template <class ViewT>
  typename boost::enable_if<IsFloatingPointIndexable<ViewT>, float>::type
  inline VHaarWavelet( ImageViewBase<ViewT> const& integral,
                       float const& x, float const& y,
                       float const& size ) {

    float response;
    float half_size = size / 2.0;
    float top = float(y) - half_size;
    float left = float(x) - half_size;

    response = -integral.impl()(left, top);
    response += integral.impl()(left+size, top);
    response += 2*integral.impl()(left, top+half_size);
    response -= 2*integral.impl()(left+size, top+half_size);
    response -= integral.impl()(left, top+size);
    response += integral.impl()(left+size, top+size);

    return response;
  }

}} // end namespace vw::ip

#endif // __VW_INTERESTPOINT_INTEGRALIMAGE_H__
