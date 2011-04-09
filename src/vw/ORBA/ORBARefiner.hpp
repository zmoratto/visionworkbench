/* 
 * File:   ORBARefiner.hpp
 * Author: hfung
 *
 * Created on April 3, 2011, 4:46 PM
 */

#ifndef ORBA_REFINER_HPP
#define	ORBA_REFINER_HPP

#include <vw/ORBA/OrbitalCameraReading.hpp>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/ORBA/ObservationSet.hpp>

namespace vw {
namespace ORBA {

using namespace vw::ba;

class ORBARefiner
{
public:

  bool refineORBAReadings(ObservationSet& obs, const Vector2& sigma_p,
          const Vector3& sigma_r, const Vector3& sigma_s, double sigma_t);
};

}} // namespace vw::ORBA


#endif	/* ORBA_REFINER_HPP */

