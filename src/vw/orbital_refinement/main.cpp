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

    // Adjust the readings
    OrbitalRefiner refinement_algorithm;
    if (!refinement_algorithm.refineOrbitalReadings(readings))
    {
        std::cerr << "Something went wrong when adjusting the orbital readings" << std::endl;
        return 1;
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

