/* 
 * File:   main.cpp
 * Author: hfung
 *
 * Created on April 2, 2011, 2:55 PM
 */

#include <vw/ORBA/OrbitalCameraReading.hpp>
#include <vw/ORBA/ObservationSet.hpp>
#include <vw/ORBA/ORBARefiner.hpp>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>
#include <fstream>
#include <list>

using namespace std;
using namespace vw::camera;
using namespace vw::ORBA;
using namespace vw::ba;

void write_results (std::string file, std::list<OrbitalCameraReading> data)
{
    // Do something nice to convert to CSV
  ofstream output( file.c_str() );
  for (std::list<OrbitalCameraReading>::iterator it = data.begin();
       it != data.end();
       it++)
  {
    output << it->mCoord << it->mCamera->camera_pose() << "\n";
  }
  
  output.close();
}

/*
 * 
 */
int main(int argc, char** argv)
{
  std::vector<std::string> camera_names;
  std::string file_name;

// Load the pinhole models
  boost::filesystem::path my_path("/a15_rev033/");
    // default construction yields past-the-end
  boost::filesystem::directory_iterator end_itr;
  for ( boost::filesystem::directory_iterator itr( my_path );
        itr != end_itr;
        ++itr )
  {
    file_name = itr->leaf().substr(itr->leaf().find_last_of(".") + 1);
    if(file_name == "pinhole")
      camera_names.push_back(itr->leaf());
  }

  ObservationSet observations;
  observations.setExpectedReadingCount(camera_names.size());

  std::vector<boost::shared_ptr<PinholeModel> > cameras;
  BOOST_FOREACH( std::string const& in, camera_names )
  {
      // Where does this really come from?
    OrbitalReading::timestamp_t timestamp = 0;
    
    boost::shared_ptr<PinholeModel> new_cam(new PinholeModel(in));
    
      // Create a reading from the camera, store it in the observations
    observations.addReading(OrbitalCameraReading("Where does the ID come from?",
                                                 timestamp, new_cam));
  }

  // Load the control network
  std::string cnet_filename = "a15_rev033/a15_rev033-single_orbit-20110317-1313.cnet";
  boost::shared_ptr<ControlNetwork> network( new ControlNetwork("a15_rev033") );
  network->read_binary( cnet_filename );

  // Preferably, we should sort the observations by time before passing into
  // the refiner

#if 0
    // This logic should be encapsulated in the ORBARefiner, not here in the
    // main function
  

    // Perform first pass with that data that is completely OR
  OrbitalRefiner first_pass;
    //first_pass.refine( data, intermediate );

 // Perform second pass with OR-BA
 //OrbitalBARefiner second_pass;
 /*second_pass.refine( cnet, intermediate, data,
                     user_input_variance_s, user_input_variance_t,
                     user_input_variance_p, user_input_variance_r );

 // Write out data into CSV
 write_results( file, data );*/

#endif

  return 0;
}


