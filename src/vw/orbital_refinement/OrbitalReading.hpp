/* 
 * File:   OrbitalReading.hpp
 * Author: hfung
 *
 * Created on January 24, 2011, 11:41 PM
 */

#ifndef ORBITAL_READING_HPP
#define ORBITAL_READING_HPP

#include <string>

struct OrbitalReading
{
public:
  // Define the data type we're using to store timestamps with
  // a millisecond resolution
  typedef unsigned long long timestamp_t;

  // Constructor
  OrbitalReading(std::string id, timestamp_t time,
    double x, double y, double z) :
    mId(id),
    mTime(time)
  {
    mCoord[0] = x;
    mCoord[1] = y;
    mCoord[2] = z;
  }

  // Destructor
  ~OrbitalReading()
  {
    delete[] mCoord;
  }

  std::string mId;
  timestamp_t mTime;
  double mCoord[3];

  struct TimestampLess
  {
    bool operator()(const OrbitalReading& lhs, const OrbitalReading& rhs)
    {  return lhs.mTime < rhs.mTime; }
  };
};



#endif

