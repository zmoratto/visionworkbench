/* 
 * File:   OrbitalReader.hpp
 * Author: hfung
 *
 * Created on January 24, 2011, 11:40 PM
 */

#ifndef ORBITAL_READER_HPP
#define ORBITAL_READER_HPP

#include <string>
#include <list>

#include "OrbitalReading.hpp"

class OrbitalReader
{
public:

  //! Read camera data from an input file in CSV format.
  //! \param input_filename The name of a CSV file to read input from
  //! \param readings The container where the readings taken from the file are stored
  //! \return true if it succeeded, false otherwise.
  bool readFromCSV(const std::string input_filename, std::list<OrbitalReading>& readings);
};

#endif

