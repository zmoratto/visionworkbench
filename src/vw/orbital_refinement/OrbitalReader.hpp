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
#include <list>

OrbitalReading::timestamp_t stringToTime(std::string time_string);

// Read camera data from an input file.
// Return a list of orbital readings
std::list<OrbitalReading> camread(const std::string input_filename);

#endif

