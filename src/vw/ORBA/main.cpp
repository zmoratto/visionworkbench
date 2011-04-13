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
#include <boost/program_options.hpp>

#include <cstdlib>
#include <fstream>
#include <list>

#include "ObservationSet.hpp"

using namespace std;
using namespace vw::camera;
using namespace vw::ORBA;
using namespace vw::ba;

void write_results (std::string file, std::vector<OrbitalCameraReading> data)
{
    // Do something nice to convert to CSV
  ofstream output( file.c_str() );
  for (std::vector<OrbitalCameraReading>::iterator it = data.begin();
       it != data.end();
       it++)
  {
    output << it->mId << "," 
           << it->mCoord[0] << ","
           << it->mCoord[1] << ","
           << it->mCoord[2] << ","
           << it->mCamera->camera_pose() << ","
           << it->mTime << "\n";
  }
  
  output.close();
}

bool read_input (std::string filename, std::vector<std::string>& camera_names,
                 std::vector<OrbitalReading::timestamp_t>& times)
{
  std::string entry_id_string, time_string;

  // Open the file
  std::ifstream input_file(filename.c_str());
  if (!input_file)
    return false;

  // Read each line.
  // Note that this code only parses the format found in our sample data,
  // where none of the fields have any of the "interesting" features of
  // CSV like embedded commas.
  std::string line_buffer;
  std::istringstream line;
  while (std::getline(input_file, line_buffer))
  {
    // Get the string ready to parse
    line.clear();
    line.str(line_buffer);

    // Read the next line of data
    std::getline(line, entry_id_string, ',');
    std::getline(line, time_string, ',');

    if (line.fail())
      break;

    camera_names.push_back(entry_id_string);

    // Not sure of the logic here?
    double time = atof(time_string.c_str());
    time_t ts = static_cast<time_t>(time);
    OrbitalReading::timestamp_t result = ts;

    times.push_back(result);
  }
  return true;
}

/*
 * 
 */
int main(int argc, char** argv)
{
  std::vector<std::string> camera_names;
  std::vector<OrbitalReading::timestamp_t> times;
  std::string file_name = "a15_rev033_ephemeris.time";

  // Setup variables to store variances
  double var_p;
  double var_r;
  double var_s;
  double var_t;

  // Setup the input options
  namespace po = boost::program_options;
  po::options_description desc("Allowed options:");
   desc.add_options()
    ("help", "Produce help message")
    ("projection", po::value<double>(&var_p)->default_value(0), "Set projection variance")
    ("registration", po::value<double>(&var_r)->default_value(0), "Set registration variance")
    ("satellite", po::value<double>(&var_s)->default_value(0), "Set satellite variance")
    ("timing", po::value<double>(&var_t)->default_value(0), "Set timing variance")
   ;

  // Setup variable map
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  // Display help message if option was called
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  read_input(file_name, camera_names, times);

  ObservationSet observations;
  ObservationSet dest;
  observations.setExpectedReadingCount(camera_names.size());

  // Counter for timestamps, should be same order as camera names
  int i = 0;

  BOOST_FOREACH( std::string const& in, camera_names )
  {
    
    boost::shared_ptr<PinholeModel> new_cam(new PinholeModel(in));
    
    // Create a reading from the camera, store it in the observations
    observations.addReading(OrbitalCameraReading(in, times[i], new_cam));

    // Increment the counter for the timestamp vector
    i++;
  }

  // Load the control network
  std::string cnet_filename = "a15_rev033/a15_rev033-single_orbit-20110317-1313.cnet";
  boost::shared_ptr<ControlNetwork> network( new ControlNetwork("a15_rev033") );
  network->read_binary( cnet_filename );

  // Now set the control network in the observation set
  observations.setControlNetwork(network);

  // Setup the variances to pass into refiner
  const Vector2 sigma_p(var_p);
  const Vector3 sigma_r(var_r);
  const Vector3 sigma_s(var_s);
  const double sigma_t = var_t;

  ORBARefiner refiner;
  refiner.refineORBAReadings(observations, dest, sigma_p, sigma_r, sigma_s, sigma_t);

  // Write to a file
  write_results("a15_rev033_refined.csv", dest.getReadings());

  return 0;
}


