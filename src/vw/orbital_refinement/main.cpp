/* 
 * File:   main.cpp
 * Author: hfung
 *
 * Created on January 25, 2011, 1:18 AM
 */
#include "OrbitalReader.hpp"
#include "OrbitalWriter.hpp"
#include "OrbitalReading.hpp"
#include "DataRefiner.hpp"
#include <cstdlib>
#include <iostream>
#include <list>

using namespace std;

void divideByOrbit(std::list< std::list<OrbitalReading> >& orbits,
          std::list<OrbitalReading> readings)
{
  // Sort the readings by timestamp.
  readings.sort(OrbitalReading::TimestampLess());

  std::list<OrbitalReading> tempOrbit;

  // Iterate through all the readings
  int k = 1;
  for (std::list<OrbitalReading>::iterator it = readings.begin();
        it != readings.end(); it++)
  {
      // If the tempOrbit is empty, add the reading we're on
      if (tempOrbit.empty())
          tempOrbit.push_back(*it);
      else
      {
          // Otherwise, check if the difference between the current reading
          // and the first reading in the orbit is within the 3300 second range (or 3300000 ms)
          // If it is, add it to the tempOrbit
          if ((it->mTime - tempOrbit.front().mTime) < 3300000 )
              tempOrbit.push_back(*it);
          // Otherwise...
          else
          {
              // Add the tempOrbit to the orbits list
              orbits.push_back(tempOrbit);
              // Clear the tempOrbit, then add the current reading as the first
              // of the next set of orbits

              /* DEBUG
              std::cout << "new orbit ----------" << std::endl;
              for (std::list<OrbitalReading>::iterator is = tempOrbit.begin();
                    is != tempOrbit.end(); is++)
              {
                  std::cout << k << ", " << is->mTime << std::endl;
                  k++;
              }
              std::cout << std::endl; */
              tempOrbit.clear();
              tempOrbit.push_back(*it);
          }
      }

  }

  // We still have the last tempOrbit, so push it back to orbits
  orbits.push_back(tempOrbit);
}

/*
 * 
 */
int main(int argc, char** argv) {
    std::string INPUT_FILE = "apollo_17_orbit_positions.csv";
    std::string OUTPUT_FILE = "apollo_17_orbit_positions_refined.csv";

    // Create a place to store the readings.
    std::list<OrbitalReading> readings;

    // Read the input
    OrbitalReader input_reader;
    if (!input_reader.readFromCSV(INPUT_FILE, readings))
    {
        std::cerr << "Something went wrong when reading the input file" << std::endl;
        return 1;
    }

    // Break the readings into orbits
    std::list< std::list<OrbitalReading> > orbits;
    divideByOrbit(orbits, readings);

    for (std::list< std::list<OrbitalReading> >::iterator read_orb = orbits.begin();
          read_orb != orbits.end(); ++read_orb)
    {
        // Adjust the readings for each orbit
        OrbitalRefiner refinement_algorithm;
        if (!refinement_algorithm.refineOrbitalReadings(*read_orb))
        {
            std::cerr << "Something went wrong when adjusting the orbital readings" << std::endl;
            return 1;
        }
    }
    

    // Write out the data back into a CSV file
    OrbitalWriter output_writer;
    if (!output_writer.writeToCSV(OUTPUT_FILE, readings))
    {
        std::cerr << "Something went wrong when writing the output file" << std::endl;
        return 1;
    }

    return 0;
}

