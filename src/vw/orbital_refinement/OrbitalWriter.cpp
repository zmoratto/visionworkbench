#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>

#include <set>

#include "OrbitalWriter.hpp"


bool camwrite(std::multiset<OrbitalReading, OrbitalReading::TimestampLess> readings,
        const std::string output_filename)
{
    std::ofstream output_stream(output_filename);

    if (!output_stream)
        return false;

    int i = 0;
    for (std::multiset<OrbitalReading, OrbitalReading::TimestampLess>::iterator it = readings.begin();
         it != readings.end();
         ++it, ++i)
    {
      std::cout << i << ", "
                << it->mId << ", "
                << it->mTime << ", "
                << it->mCoord[0] << ", "
                << it->mCoord[1] << ", "
                << it->mCoord[2] << "\n";
    }

    output_stream.close();

    return true;
}