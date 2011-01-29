/* 
 * File:   DataRefiner.hpp
 * Author: hfung
 *
 * Created on January 27, 2011, 9:08 PM
 */

#ifndef DATAREFINER_HPP
#define	DATAREFINER_HPP

#include <list>
#include "OrbitalReading.hpp"

//! Adjusts a set of OrbitalReadings so that they adhere to a
//! likely orbital path.
class OrbitalRefiner
{
public:
  //! Adjust a set of OrbitalReadings so that they adhere to a
  //! likely orbital path.
  bool refineOrbitalReadings(std::list<OrbitalReading>& readings);
};


#endif	/* DATAREFINER_HPP */

