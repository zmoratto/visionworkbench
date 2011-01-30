#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>

#include <list>

#include "OrbitalWriter.hpp"

static std::string timeToString(OrbitalReading::timestamp_t);

bool OrbitalWriter::writeToCSV(const std::string output_filename,
        std::list<OrbitalReading> readings)
{
    // Create the output stream
    std::ofstream output_stream;
    output_stream.open((char *)output_filename.c_str());

    // Check that it was opened successfully
    if (!output_stream)
        throw std::string("File could not be opened for writing");

    readings.sort(OrbitalReading::TimestampLess());

    // Write to the file
    int i = 0;
    for (std::list<OrbitalReading>::iterator it = readings.begin();
         it != readings.end(); ++it, ++i)
    {
      output_stream << i << ", "
                    << it->mId << ", "
                    << it->mTime << ", "
                    << it->mCoord[0] << ", "
                    << it->mCoord[1] << ", "
                    << it->mCoord[2] << "\n";
    }

    // Close the output stream
    output_stream.close();

    return true;
}

std::string timeToString(OrbitalReading::timestamp_t time) {
    std::string str;

    // this is the time we will actually modify to get the string
    long t_time;

    // Set the prefix
    str += "APOLLO17/METRIC/";

    // Get the milliseconds
    int millis = t_time%1000;
    t_time = t_time/1000;

    // Get the seconds
    int secs = t_time%60;
    t_time = t_time/60;

    // Get the minutes
    int mins = t_time%60;
    t_time = t_time/60;

    // Get the hours
    int hours = t_time%24;
   
}