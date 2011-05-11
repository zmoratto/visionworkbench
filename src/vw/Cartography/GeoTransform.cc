// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <vw/Cartography/detail/BresenhamLine.h>
#include <vw/Cartography/GeoTransform.h>

// Vision Workbench
#include <vw/Image/ImageView.h>

// Proj.4
#include <projects.h>

namespace vw {
namespace cartography {

  // Constructor
    GeoTransform::GeoTransform(GeoReference const& src_georef, GeoReference const& dst_georef) :
    m_src_georef(src_georef), m_dst_georef(dst_georef) {
    const std::string src_datum = m_src_georef.datum().proj4_str();
    const std::string dst_datum = m_dst_georef.datum().proj4_str();

    // This optimizes in the common case where the two images are
    // already in the same map projection, and we need only apply
    // the affine transform.  This will break, of course, as soon as
    // we have mare than one type of GeoReference object, but it
    // makes life faster for now. -mbroxton
    if (m_src_georef.proj4_str() == m_dst_georef.proj4_str())
      m_skip_map_projection = true;
    else
      m_skip_map_projection = false;

    // This optimizes the case where the two datums are the same,
    // and thus we don't need to call proj to convert between them
    // as we transform.
    if(src_datum == dst_datum) {
      m_skip_datum_conversion = true;
    } else {
      // Set up the various variables for proj.
      m_skip_datum_conversion = false;

      // The source proj4 context.
      std::stringstream ss_src;
      // We convert lat/long to lat/long regardless of what the
      // source or destination georef uses.
      ss_src << "+proj=latlong " << src_datum;
      m_src_datum = boost::shared_ptr<ProjContext>(new ProjContext(ss_src.str()));
      CHECK_PROJ_INIT_ERROR(ss_src.str().c_str());

      // The destination proj4 context.
      std::stringstream ss_dst;
      ss_dst << "+proj=latlong " << dst_datum;
      m_dst_datum = boost::shared_ptr<ProjContext>(new ProjContext(ss_dst.str()));
      CHECK_PROJ_INIT_ERROR(ss_dst.str().c_str());
    }
    // Because GeoTransform is typically very slow, we default to a tolerance
    // of 0.1 pixels to allow ourselves to be approximated.
    set_tolerance( 0.1 );
  }

  // Performs a forward or reverse datum conversion.
  Vector2 GeoTransform::datum_convert(Vector2 const& v, bool forward) const {
    double x = v[0];
    double y = v[1];
    double z = 0;

    if(forward)
      pj_transform(m_src_datum->proj_ptr(), m_dst_datum->proj_ptr(), 1, 0, &x, &y, &z);
    else
      pj_transform(m_dst_datum->proj_ptr(), m_src_datum->proj_ptr(), 1, 0, &x, &y, &z);
    CHECK_PROJ_ERROR;

    return Vector2(x, y);
  }

  class Line {
    private:
      const Vector2& stop;
      Vector2 i, step;
    public:
      Line(const Vector2& start_, const Vector2& stop_)
        : stop(stop_), i(start_)
      {
        Vector2 diff = stop_-start_;
        // TODO: Is this justifiable?
        step = diff / (std::max<double>(::fabs(diff[0]), ::fabs(diff[1])));
      }

      operator bool() const { return i[0] < stop[0] && i[1] < stop[1]; }
      const vw::Vector2& operator*() const { return i; }
      Line& operator++() { i += step; return *this;}
  };

  BBox2 GeoTransform::forward_bbox( BBox2 const& bbox ) const {
    BBox2 r = TransformHelper<GeoTransform,ContinuousFunction,ContinuousFunction>::forward_bbox(bbox);

    Line l1(bbox.min(), bbox.max());
    Line l2(bbox.min() + Vector2(bbox.width(), 0),
            bbox.max() - Vector2(bbox.width(), 0));

    // these loops INTENTIONALLY skip the first step. the TransformHelper above
    // already got those points.
    while(++l1) {
      try {
        r.grow( this->forward( *l1 ) );
      } catch ( cartography::ProjectionErr const& e ) {}
    }

    while(++l2) {
      try {
        r.grow( this->forward( *l2 ) );
      } catch ( cartography::ProjectionErr const& e ) {}
    }

    return r;
  }

  BBox2 GeoTransform::reverse_bbox( BBox2 const& bbox ) const {
    BBox2 r = TransformHelper<GeoTransform,ContinuousFunction,ContinuousFunction>::reverse_bbox(bbox);

    Line l1(bbox.min(), bbox.max());
    Line l2(bbox.min() + Vector2(bbox.width(), 0),
            bbox.max() - Vector2(bbox.width(), 0));

    // these loops INTENTIONALLY skip the first step. the TransformHelper above
    // already got those points.
    while(++l1) {
      try {
        r.grow( this->reverse( *l1 ) );
      } catch ( cartography::ProjectionErr const& e ) {}
    }

    while(++l2) {
      try {
        r.grow( this->reverse( *l2 ) );
      } catch ( cartography::ProjectionErr const& e ) {}
    }

    return r;
  }

  void reproject_point_image(ImageView<Vector3> const& point_image,
                             GeoReference const& src_georef,
                             GeoReference const& dst_georef) {

    GeoTransform gtx(src_georef, dst_georef);

    // Iterate over the image, transforming the first two coordinates
    // in the Vector one at a time.  The third coordinate is taken to
    // be the altitude value, and this value is not touched.
    for (int32 j=0; j < point_image.rows(); ++j) {
      for (int32 i=0; i < point_image.cols(); ++i) {
        if (point_image(i,j) != Vector3()) {
          Vector2 in(point_image(i,j)[0], point_image(i,j)[1]);
          Vector2 out = gtx.forward(in);
          point_image(i,j).x() = out[0];
          point_image(i,j).y() = out[1];
        }
      }
    }
  }


}} // namespace vw::cartography

