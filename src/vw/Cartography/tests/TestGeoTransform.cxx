// __BEGIN_LICENSE__
//  Copyright (c) 2006-2012, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


// TestGeoTransform.h
#include <gtest/gtest_VW.h>
#include <test/Helpers.h>

#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoTransform.h>

#if defined(VW_HAVE_PKG_GDAL) && VW_HAVE_PKG_GDAL==1
#include "ogr_spatialref.h"
#endif

using namespace vw;
using namespace vw::cartography;
using namespace vw::test;

TEST( GeoTransform, BasicTransform ) {
  GeoReference src_georef;
  src_georef.set_well_known_geogcs("WGS84");

  GeoReference dst_georef;
  dst_georef.set_well_known_geogcs("WGS84");

  GeoTransform geotx(src_georef,dst_georef);

  Vector2 fwd = geotx.forward(Vector2(25,25)),
    rev = geotx.reverse(Vector2(25,25));

  EXPECT_VECTOR_NEAR( fwd, Vector2(25,25), 1e-16 );
  EXPECT_VECTOR_NEAR( rev, Vector2(25,25), 1e-16 );
}

TEST( GeoTransform, UTMFarZone ) {
  // This tests for a bug where forward_bbox calls latlon_to_* for a latlon
  // that is invalid for a utm zone.

  std::vector<Vector2> utm(4);
  utm[0] = Vector2(419832.648, 5184829.285); // UL
  utm[1] = Vector2(419832.648, 5129329.285); // LL
  utm[2] = Vector2(469332.648, 5184829.285); // UR
  utm[3] = Vector2(469332.648, 5129329.285); // LR

  Vector2i size(3300,3700);

  GeoReference ll_georef, utm_georef;

  Matrix3x3 utm_map;
  utm_map(0,0) =  (utm[2][0] - utm[0][0]) / static_cast<double>(size(0));
  utm_map(1,1) = -(utm[0][1] - utm[1][1]) / static_cast<double>(size(1));
  utm_map(0,2) = utm[0][0];
  utm_map(1,2) = utm[0][1];
  utm_map(2,2) = 1;
  utm_georef.set_transform(utm_map);
  utm_georef.set_pixel_interpretation(GeoReference::PixelAsPoint);
  // pick one far away from the poles
  utm_georef.set_UTM(59, false); // 59S

  Matrix3x3 ll_map;
  ll_map(0,0) =  360.0;
  ll_map(1,1) = -180.0;
  ll_map(0,2) = -180;
  ll_map(1,2) = 90;
  ll_map(2,2) = 1;
  ll_georef.set_transform(ll_map);

  GeoTransform geotx(utm_georef, ll_georef);

  BBox2i bbox, bbox2;

  // This should work
  EXPECT_NO_THROW( bbox = geotx.forward_bbox(BBox2i(0,0,size[0],size[1])) );

  // This should throw a proj.4 error because it tries to look up a lonlat
  // that is outside the utm zone
  EXPECT_THROW( bbox2 = geotx.reverse_bbox(BBox2i(0,0,size[0],size[1])) ,
                vw::ArgumentErr );
}

TEST( GeoTransform, StereographicSingularity ) {
  // Test forward bbox actually hits the limit of platecarre
  GeoReference ll_georef, stereo_georef;

  Matrix3x3 transform = math::identity_matrix<3>();
  transform(1,1) = -1; transform(1,2) = 90;
  ll_georef.set_transform(transform);

  transform = math::identity_matrix<3>();
  transform(0,0) = transform(1,1) = 1e4;
  transform(0,2) = transform(1,2) = -1e6;
  stereo_georef.set_transform(transform);
  stereo_georef.set_stereographic(90,0,1);

  GeoTransform geotx(stereo_georef, ll_georef);
  BBox2i output, input(50,50,100,100);
  EXPECT_NO_THROW( output = geotx.forward_bbox(input) );
  EXPECT_NEAR( 0, output.min()[1], 2 );
}

void load_proj4( std::string const& proj4,
                 GeoReference& georef ) {
  OGRSpatialReference gdal_spatial_ref;
  if (gdal_spatial_ref.SetFromUserInput( proj4.c_str() ))
    vw_throw( ArgumentErr() << "Failed to parse: \"" << proj4 << "\"." );
  char *wkt = NULL;
  gdal_spatial_ref.exportToWkt( &wkt );
  std::string wkt_string(wkt);
  delete[] wkt;

  georef.set_wkt( wkt_string );
}

TEST( GeoTransform, DatumTransition ) {
  GeoReference georef1, georef2, georef3;
  load_proj4("+proj=lcc +lat_1=38.43333333333333 +lat_2=37.06666666666667 +lat_0=36.5 +lon_0=-120.5 +x_0=609601.2192024384 +y_0=0 +ellps=clrk66 +datum=NAD27 +to_meter=0.34 +no_defs", georef1);
  georef1.set_transform(math::identity_matrix<3>());
  load_proj4("+proj=lcc +lat_1=38.43333333333333 +lat_2=37.06666666666667 +lat_0=36.5 +lon_0=-120.5 +x_0=2000000 +y_0=500000 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs",georef2);
  Matrix3x3 transform = math::identity_matrix<3>();
  transform(0,0) = 0.5;
  transform(1,1) = -0.5;
  georef2.set_transform(math::identity_matrix<3>());
  load_proj4("+proj=sinu +lon_0=0 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs ",georef3);
  transform(0,2) = 10000;
  transform(1,2) = -1000000;
  georef3.set_transform(transform);

  GeoReference georef2_ll;
  load_proj4("+proj=longlat +units=m",georef2_ll);
  georef2_ll.set_transform(math::identity_matrix<3>());

  // Comparing 1->2 to 1->3->2. They should produce the same results if everything worked
  GeoTransform geotx12( georef1, georef2 );
  GeoTransform geotx13( georef1, georef3 );
  GeoTransform geotx32( georef3, georef2 );

  for ( size_t x = 0; x < 10000000; x += 100000 ) {
    for ( size_t y = 0; y < 10000000; y+= 100000 ) {
      // Verify GeoTransforms are consistent
      EXPECT_VECTOR_NEAR( geotx12.forward( Vector2(x,y) ),
                          geotx32.forward(geotx13.forward( Vector2(x,y) )), 1e-6 );

      // Verify that we're consistent with GeoRefs
      Vector2 lonlat1 = georef1.pixel_to_lonlat( Vector2(x,y) ); // it appears like lonlat is in geocentric coordinates.
      EXPECT_VECTOR_NEAR( geotx12.forward( Vector2(x,y) ),
                          georef2.lonlat_to_pixel(lonlat1), 1e-6 );
    }
  }
}
