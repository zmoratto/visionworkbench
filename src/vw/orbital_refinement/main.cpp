/* 
 * File:   main.cpp
 * Author: hfung
 *
 * Created on January 25, 2011, 1:18 AM
 */
#include "OrbitalReader.hpp"
#include "OrbitalWriter.hpp"
#include "OrbitalReading.hpp"
#include <cstdlib>

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

    // Create the custom data store
    std::multiset<OrbitalReading, OrbitalReading::TimestampLess> readings;

    // Read in the coordinate file
    camread("apollo_17_orbit_positions.csv", readings);




    // Write the updated coordinates
    camwrite(readings, "apollo_17_orbit_outfile.csv");

    return 0;
}

