/* 
 * File:   DataRefiner.hpp
 * Author: hfung
 *
 * Created on January 27, 2011, 9:08 PM
 */

#ifndef DATAREFINER_HPP
#define	DATAREFINER_HPP

#include <list>
#include <vector>
#include "OrbitalReading.hpp"

//! Adjusts a set of OrbitalReadings so that they adhere to a
//! likely orbital path.
class OrbitalRefiner
{
public:
  //! Adjust a set of OrbitalReadings so that they adhere to a
  //! likely orbital path.
  bool refineOrbitalReadings(std::list<OrbitalReading>& readings,
          std::list<OrbitalReading>& refined);

  double getCalculatedGM() const
  { return mGM; }

  vw::Vector3 getCalculatedInitialPosition() const
  { return mP0; }

  vw::Vector3 getCalculatedInitialVelocity() const
  { return mV0; }

  const std::vector<double>& getInlierWeights() const
  { return mWeights; }

  std::vector<double>& getInlierWeights()
  { return mWeights; }

private:
  
  double mGM;
  vw::Vector3 mP0;
  vw::Vector3 mV0;
  std::vector<double> mWeights;
};


#endif	/* DATAREFINER_HPP */

