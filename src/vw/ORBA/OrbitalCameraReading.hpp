/* 
 * File:   OrbitalCameraReading.h
 * Author: hfung
 *
 * Created on April 1, 2011, 4:57 PM
 */

#ifndef ORBITAL_CAMERA_READING_H
#define ORBITAL_CAMERA_READING_H

#include <vw/orbital_refinement/OrbitalReading.hpp>
#include <vw/Camera/PinholeModel.h>

namespace vw {
namespace ORBA {

using namespace vw::camera;
    
// A single reading/image.  It has a location,
// a timestamp, and ID, a camera model, and an
// orientation (pose).
struct OrbitalCameraReading : public OrbitalReading
{
  boost::shared_ptr<PinholeModel> mCamera;
  
  OrbitalCameraReading( std::string id, timestamp_t time,
                        boost::shared_ptr<PinholeModel> cam ) 
          : OrbitalReading(id, time, cam->camera_center()),
            mCamera(cam)
  {}
};

}}

#endif	/* ORBITAL_CAMERA_READING_H */
