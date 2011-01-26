/* 
 * File:   OrbitalReader.hpp
 * Author: hfung
 *
 * Created on January 24, 2011, 11:40 PM
 */

#ifndef ORBITAL_READER_HPP
#define ORBITAL_READER_HPP

#include "OrbitalReading.hpp"
#include <string>
#include <set>

OrbitalReading::timestamp_t stringToTime(std::string time_string);

// Read camera data from an input file.
// Return a set of orbital readings
//std::multiset<OrbitalReading, OrbitalReading::TimestampLess>
bool camread(const std::string input_filename,
        std::multiset<OrbitalReading, OrbitalReading::TimestampLess> readings);

#endif

