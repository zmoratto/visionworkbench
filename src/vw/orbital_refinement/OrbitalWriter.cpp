#include <fstream>
#include <iostream>
#include <iomanip>
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

      output_stream << it->mId << ", "
                    << timeToString(it->mTime) << ", "
                    << std::setprecision(12)
                    << it->mCoord[0] << ", "
                    << std::setprecision(12)
                    << it->mCoord[1] << ", "
                    << std::setprecision(12)
                    << it->mCoord[2] << "\n";
    }

    // Close the output stream
    output_stream.close();

    return true;
}

std::string timeToString(OrbitalReading::timestamp_t time)
{
    char buffer[256];

    // Get the milliseconds, then convert to seconds
    int milliseconds = time%1000;
    time_t temp_time = time/1000;

    // Convert to a struct tm in UTC
    struct tm* t = localtime(&temp_time);

    int year = t->tm_year + 1900;
    int month = t->tm_mon + 1;
    int day = t->tm_mday;
    int hour = t->tm_hour;
    int min = t->tm_min;
    int secs = t->tm_sec;

    sprintf(buffer, "APOLLO17/METRIC/%.4u-%.2u-%.2uT%.2u:%.2u:%.2u.%.3u",
            year, month, day, hour, min, secs, milliseconds);

    std::string str = buffer;

    return str;
}
