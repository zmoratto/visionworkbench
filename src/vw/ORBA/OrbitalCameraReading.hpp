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
// a timestamp, and ID, a camera model, an
// orientation (pose), and a numeric image ID.
struct OrbitalCameraReading : public OrbitalReading
{
  boost::shared_ptr<PinholeModel> mCamera;
  std::size_t mImageID;
  
  OrbitalCameraReading( std::string id, timestamp_t time,
                        boost::shared_ptr<PinholeModel> cam,
                        std::size_t image_id) 
          : OrbitalReading(id, time, cam->camera_center()),
            mCamera(cam), mImageID(image_id)
  {}
};

}}

#endif	/* ORBITAL_CAMERA_READING_H */
