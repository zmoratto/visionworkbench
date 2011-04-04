/* 
 * File:   ORBARefiner.hpp
 * Author: hfung
 *
 * Created on April 3, 2011, 4:46 PM
 */

#ifndef ORBA_REFINER_HPP
#define	ORBA_REFINER_HPP

#include <OrbitalCameraReading.hpp>

using namespace vw::ba;

class ORBARefiner
{
public:

  bool refineORBAReadings(ControlNetwork& cnet,
          std::list<OrbitalCameraReading>& readings,
          std::list<OrbitalCameraReading>& refined);
};

#endif	/* ORBA_REFINER_HPP */

