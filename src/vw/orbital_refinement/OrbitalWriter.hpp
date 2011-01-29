/* 
 * File:   OrbitalWriter.hpp
 * Author: hfung
 *
 * Created on January 25, 2011, 1:13 AM
 */

#ifndef ORBITAL_WRITER_HPP
#define	ORBITAL_WRITER_HPP

#include "OrbitalReading.hpp"
#include <string>
#include <list>

std::string timeToString(OrbitalReading::timestamp_t);

bool camwrite(const std::string output_filename,
        std::list<OrbitalReading> readings);

#endif	/* ORBITALWRITER_HPP */

