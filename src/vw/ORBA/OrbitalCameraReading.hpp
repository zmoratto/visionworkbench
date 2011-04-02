/* 
 * File:   OrbitalCameraReading.h
 * Author: hfung
 *
 * Created on April 1, 2011, 4:57 PM
 */

#ifndef ORBITAL_CAMERA_READING_H
#define	ORBITAL_CAMERA_READING_H

#include <vw/orbital_refinement/OrbitalReading.hpp>

using namespace vw::math;
using namespace vw::camera;

struct OrbitalCameraReading : public OrbitalReading {
 OrbitalCameraReading( std::string id, timestamp_t time,
                       boost::shared_ptr<PinholeModel> cam ) {
  mCoord = cam.camera_center(Vector2()); // We are assuming pinhole model
  mQuat = cam.camera_pose(Vector2());
  mId = id;
  mTime = time;

 }
 boost::shared_ptr<CameraModel> mCamera;
 Quaternion mQuat;
}


#endif	/* ORBITAL_CAMERA_READING_H */