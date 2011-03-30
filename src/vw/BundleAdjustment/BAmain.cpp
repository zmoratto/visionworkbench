/* 
 * File:   BAmain.cpp
 * Author: hfung
 *
 * Created on March 19, 2011, 3:57 PM
 */
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/BACGModel.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <cstdlib>

using namespace std;
using namespace vw::camera;
using namespace vw::ba;
using namespace vw::math;

namespace vw {

/*
 * 
 */
int main(int argc, char** argv) {
    // Staging
  typedef boost::shared_ptr<AdjustedCameraModel> AdjCamObj;
  std::vector<std::string> camera_names;
  std::string fileName;

  boost::filesystem::path my_path("/a15_rev033/");
  directory_iterator end_itr; // default construction yields past-the-end
  for ( directory_iterator itr( my_path );
      itr != end_itr;
      ++itr )
      {
         fileName = itr->leaf().substr(itr->leaf().find_last_of(".") + 1);
         if(fileName == "pinhole")
            camera_names.push_back(itr->leaf());
      }


  std::vector<boost::shared_ptr<PinholeModel> > cameras;
  BOOST_FOREACH( std::string const& in, camera_names ) {
    cameras.push_back( boost::shared_ptr<PinholeModel>( new PinholeModel(in) ) );
  }
  
  std::string cnet_filename = "a15_rev033/a15_rev033-single_orbit-20110317-1313.cnet";
  boost::shared_ptr<ControlNetwork> network( new ControlNetwork("a15_rev033") );
  network->read_binary( cnet_filename );

  Vector a_delta = or_return - cameras[0].camera_center();

  // New vector adjust camera model that use current As!
  boost::shared_ptr<AdjustedCameraModel> pin(PinholeModel("AS15-M-1015.lev2.pinhole"));
  AdjustedCameraModel q( pin, a_delta,
                         Quat a_secondhalf(0,0,0,1) );

  double error = 0;
  BOOST_FOREACH( ControlPoint const& cp,
                 cnet ) {
    Vector3 b = cp.position();

    BOOST_FOREACH( ControlMeasure const& cm, cp ) {
      AdjCamObj q = cameras[cm.image_id()];
      error += norm_2( cm.position() - q->point_to_pixel(b) );
    }
  }
  std::cout << "Error: " << error << "\n";

    return 0;
}

}
