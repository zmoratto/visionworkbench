/* 
 * File:   main.cpp
 * Author: hfung
 *
 * Created on January 25, 2011, 1:18 AM
 */
#include "OrbitalReader.hpp"
#include "OrbitalReading.hpp"
#include "DataRefiner.hpp"
#include <cstdlib>
#include <iostream>
#include <list>

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    // Declare a list
    std::list<OrbitalReading> readings;

    // Read in the data from the CSV file
    try
    {
       readings = camread("apollo_17_orbit_positions.csv");
    }
    catch(std::string error)
    {
       std::cout << "Exception caught: " << error;
    }

    // Send readings through for orbital refinement
    refineData(readings);

    // Write out the data back into a CSV file
    // camwrite("apollo_17_orbit_positions_refined.csv");

    return 0;
}

