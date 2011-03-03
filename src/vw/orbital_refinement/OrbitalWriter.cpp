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
                    << std::setiosflags(std::ios::fixed) << std::setprecision(5)
                    << it->mCoord[0] << ", "
                    << std::setiosflags(std::ios::fixed) << std::setprecision(6)
                    << it->mCoord[1] << ", "
                    << it->mCoord[2] << "\n";

    }

    // Close the output stream
    output_stream.close();

    return true;
}

std::string timeToString(OrbitalReading::timestamp_t time) {

    char buffer[256];

    // this is the time we will actually modify to get the string
    time_t temp = (time_t)time;

    // Get the milliseconds, then convert to seconds
    int milliseconds = temp%1000;
    time_t tempTime = temp/1000;

    // Convert to a struct tm in UTC
    struct tm* t = localtime(&tempTime);

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