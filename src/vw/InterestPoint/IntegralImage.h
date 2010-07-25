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
  template <class ViewT>
  inline typename ViewT::pixel_type IntegralBlock( ImageViewBase<ViewT> const& view,
                                                   BBox2i const& area ) {
    // Shifting for definition change of integral image
    BBox2i iarea( area.min()-Vector2i(1,1), area.size() );
    ViewT const& integral = view.impl();

#if defined(VW_ENABLE_BOUNDS_CHECK) && (VW_ENABLE_BOUNDS_CHECK==1)
    VW_ASSERT( bounding_box(integral).contains( iarea.min() ),
               vw::ArgumentErr() << "Min of area is out of bounds. Accessing "
               << iarea.min() << "\n" );
    VW_ASSERT( bounding_box(integral).contains( area.max() ),
               vw::ArgumentErr() << "Max of area is out of bounds. Accessing "
               << area.max() << "\n" );
#endif

    return integral( iarea.min()[0], iarea.min()[1] ) +
      integral(iarea.max()[0], iarea.max()[1] ) -
      integral(iarea.max()[0], iarea.min()[1] ) -
      integral(iarea.min()[0], iarea.max()[1] );
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
  template <class ViewT>
  inline typename ViewT::pixel_type
  XSecondDerivative( ImageViewBase<ViewT> const& view,
                     int const& x, int const& y,
                     unsigned const& filter_size ) {
    VW_ASSERT( filter_size % 2 == 1,
               vw::LogicErr() << "XSecondDerivative's filter size must be odd" );
    ViewT const& integral = view.impl();
    unsigned lobe = filter_size / 3;
    unsigned half_lobe = (unsigned) floor( float(lobe) / 2.0 );
    typename ViewT::pixel_type derivative;

    // Adding positive left
    derivative = IntegralBlock( integral,
                                BBox2i( x - lobe - half_lobe, y - lobe + 1,
                                        lobe, 2*lobe - 1 ) );

    // Adding negative middle;
    derivative -= 2.0*IntegralBlock( integral,
                                     BBox2i( x - half_lobe, y - lobe + 1,
                                             2*half_lobe+1, 2*lobe - 1 ) );

    // Adding positive right
    derivative += IntegralBlock( integral,
                                 BBox2i( x + half_lobe + 1, y - lobe + 1,
                                         lobe, 2*lobe - 1 ) );

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
  template <class ViewT>
  inline typename ViewT::pixel_type
  YSecondDerivative( ImageViewBase<ViewT> const& view,
                     int const& x, int const& y,
                     unsigned const& filter_size ) {
    VW_ASSERT( filter_size % 2 == 1,
               vw::LogicErr() << "YSecondDerivative's filter size must be odd" );
    ViewT const& integral = view.impl();
    unsigned lobe = filter_size / 3;
    unsigned half_lobe = (unsigned) floor( float(lobe) / 2.0 );
    typename ViewT::pixel_type derivative;

    // Adding positive top;
    derivative = IntegralBlock( integral,
                                BBox2i( x - lobe + 1, y - lobe - half_lobe,
                                        2*lobe - 1, lobe ) );

    // Adding negative middle;
    derivative -= 2.0*IntegralBlock( integral,
                                     BBox2i( x - lobe + 1, y - half_lobe,
                                             2*lobe - 1, 2*half_lobe+1 ) );

    // Adding positive bottom;
    derivative += IntegralBlock( integral,
                                 BBox2i( x - lobe + 1, y + half_lobe + 1,
                                         2*lobe - 1, lobe ) );

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
    VW_ASSERT( filter_size % 2 == 1,
               vw::LogicErr() << "XYDerivative's filter size must be odd" );
    unsigned lobe = filter_size / 3;
    PixelT derivative;

    // Adding positive top left
    derivative = IntegralBlock( integral,
                                BBox2i( x - lobe, y - lobe, lobe, lobe ) );

    // Adding negative top right
    derivative -= IntegralBlock( integral,
                                 BBox2i( x + 1, y - lobe, lobe, lobe ) );

    // Adding negative bottom left
    derivative -= IntegralBlock( integral,
                                 BBox2i( x - lobe, y + 1, lobe, lobe ) );

    // Adding positve bottom right
    derivative += IntegralBlock( integral,
                                 BBox2i( x + 1, y + 1, lobe, lobe ) );

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
  inline HHaarWavelet( ImageViewBase<ViewT> const& view,
                       int const& x, int const& y,
                       float const& size ) {
    VW_ASSERT( int(size) % 2 == 0,
               vw::LogicErr() << "HHaarWavelet's filter size must be even" );
    ViewT const& integral = view.impl();
    float response;
    int half_size = int(round( size / 2.0));
    int i_size = half_size << 1;
    int top = int(round( y - size/2));
    int left = int(round( x - size/2));

#if defined(VW_ENABLE_BOUNDS_CHECK) && (VW_ENABLE_BOUNDS_CHECK==1)
    VW_ASSERT(left+i_size < integral.cols(),
              vw::ArgumentErr() << "left out of bounds. "<< integral.cols() <<" : "
              << left+i_size << " [top left] " << top << " " << left << "\n");
    VW_ASSERT(top+i_size < integral.rows(),
              vw::ArgumentErr() << "top out of bounds. " << integral.rows() <<" : "
              << top+i_size << "\n");
    VW_ASSERT(left >= 0,
              vw::ArgumentErr() << "left is too low. " << 0 << " : " << left << "\n");
    VW_ASSERT(top >= 0,
              vw::ArgumentErr() << "top is too low. " << 0 << " : " << top << "\n");
#endif

    response = -integral(left, top);
    response += 2*integral(left+half_size, top);
    response -= integral(left+i_size, top);
    response += integral(left, top+i_size);
    response -= 2*integral(left+half_size, top+i_size);
    response += integral(left+i_size, top+i_size);

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
  inline HHaarWavelet( ImageViewBase<ViewT> const& view,
                       float const& x, float const& y,
                       float const& size ) {
    ViewT const& integral = view.impl();
    float response;
    float half_size = size / 2.0;
    float top = y - half_size;
    float left = x - half_size;

    response = -integral(left, top);
    response += 2*integral(left+half_size, top);
    response -= integral(left+size, top);
    response += integral(left, top+size);
    response -= 2*integral(left+half_size, top+size);
    response += integral(left+size, top+size);

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
  inline VHaarWavelet( ImageViewBase<ViewT> const& view,
                       int const& x, int const& y,
                       float const& size ) {
    VW_ASSERT( int(size) % 2 == 0,
               vw::LogicErr() << "VHaarWavelet's filter size must be odd" );
    ViewT const& integral = view.impl();
    float response;
    int half_size = int(round( size / 2.0));
    int i_size = half_size << 1;
    int top = int(round( y - size/2));
    int left = int(round( x - size/2));

#if defined(VW_ENABLE_BOUNDS_CHECK) && (VW_ENABLE_BOUNDS_CHECK==1)
    VW_ASSERT(left+i_size < integral.cols(),
              vw::ArgumentErr() << "left out of bounds. "<< integral.cols() <<" : "
              << left+i_size << " [top left] " << top << " " << left << "\n");
    VW_ASSERT(top+i_size < integral.rows(),
              vw::ArgumentErr() << "top out of bounds. " << integral.rows() <<" : "
              << top+i_size << "\n");
    VW_ASSERT(left >= 0,
              vw::ArgumentErr() << "left is too low. " << 0 << " : " << left << "\n");
    VW_ASSERT(top >= 0,
              vw::ArgumentErr() << "top is too low. " << 0 << " : " << top << "\n");
#endif

    response = -integral(left, top);
    response += integral(left+i_size, top);
    response += 2*integral(left, top+half_size);
    response -= 2*integral(left+i_size, top+half_size);
    response -= integral(left, top+i_size);
    response += integral(left+i_size, top+i_size);

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
  inline VHaarWavelet( ImageViewBase<ViewT> const& view,
                       float const& x, float const& y,
                       float const& size ) {
    ViewT const& integral = view.impl();
    float response;
    float half_size = size / 2.0;
    float top = float(y) - half_size;
    float left = float(x) - half_size;

    response = -integral(left, top);
    response += integral(left+size, top);
    response += 2*integral(left, top+half_size);
    response -= 2*integral(left+size, top+half_size);
    response -= integral(left, top+size);
    response += integral(left+size, top+size);

    return response;
  }

}} // end namespace vw::ip

#endif // __VW_INTERESTPOINT_INTEGRALIMAGE_H__
