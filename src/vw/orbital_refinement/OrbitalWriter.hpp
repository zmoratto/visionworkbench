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
#include <set>

bool camwrite(std::multiset<OrbitalReading, OrbitalReading::TimestampLess> readings,
        const std::string output_filename);

#endif	/* ORBITALWRITER_HPP */

