/* 
 * File:   OrbitalReading.hpp
 * Author: hfung
 *
 * Created on January 24, 2011, 11:41 PM
 */

#ifndef ORBITAL_READING_HPP
#define ORBITAL_READING_HPP

#include <string>
#include <vw/Math/Vector.h>

struct OrbitalReading
{
public:
  // Define the data type we're using to store timestamps with
  // a millisecond resolution
  typedef vw::int64 timestamp_t;

  // Constructor
  OrbitalReading(std::string id, timestamp_t time,
    double x, double y, double z) :
    mId(id),
    mTime(time),
    mCoord(x,y,z)
  {}

  std::string mId;
  timestamp_t mTime;
  vw::Vector3 mCoord;

  struct TimestampLess
  {
    bool operator()(const OrbitalReading& lhs, const OrbitalReading& rhs)
    {  return lhs.mTime < rhs.mTime; }
  };
};



#endif

