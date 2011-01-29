#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>

#include <list>

#include "OrbitalReader.hpp"

// This function reads from the input file, creating a set of reading objects
// with an ID, normalized timestamp, x, y, and z.
std::list<OrbitalReading> camread(const std::string input_filename)
{
  double x, y, z;
  std::string entry_id_string, time_string;

  // Open the file
  std::ifstream input_file(input_filename.c_str());
  if (!input_file)
    throw std::string("File could not be opened"); 

  // Create a place to store the readings.  I chose a std::multiset<>
  // so that it stays sorted by timestamp as we add the entries.  I chose
  // multiset<> over set<> in case two entries have the same timestamp.
  std::list<OrbitalReading> readings;

  // Read each line.
  // Note that this code only parses the format found in our sample data,
  // where none of the fields have any of the "interesting" features of
  // CSV like embedded commas.
  std::string line_buffer;
  std::istringstream line;
  while (std::getline(input_file, line_buffer))
  {
    // Get the string ready to parse
    line.clear();
    line.str(line_buffer);

    // The dummy char 'c' reads the comma characters.
    char c;

    // Read the next line of data
    std::getline(line, entry_id_string, ',');
    std::getline(line, time_string, ',');
    line >> x >> c
         >> y >> c
         >> z;
    // If we didn't get good data, skip it.
    // Probably need more robust distinction between blank lines and
    // bad input.
    if (line.fail())
      break;

    // Convert the time string to a numeric value representing the
    // integer number of milliseconds since 1970.
    OrbitalReading::timestamp_t t = stringToTime(time_string);

    // Create a new reading based on the data we just read.
    readings.push_back(OrbitalReading(entry_id_string, t, x, y, z));
  }

  // They are already sorted by time (because we used a multiset).
  // Now normalize the time readings by subtracting the lowest time
  // from all of the timestamps.
/*
   PLB: std::sets are immutable
      See (http://stackoverflow.com/questions/3775414/assignment-of-data-member-in-read-only-structure-class-in-stl-set) 
      for more discussion. I'd suggest a different container type if you intend to do much manipulation of its contained objects
      
      Unfortunately sorted containers usually lock the key, requiring you to remove and reinsert objects if their keys change.
*/
    OrbitalReading::timestamp_t min_time = readings.begin()->mTime;
  for (std::list<OrbitalReading>::iterator it = readings.begin();
       it != readings.end();
       ++it)
  {
    it->mTime -= min_time;
  }

  // Now for fun, write out the readings
  int i = 0;
  for (std::list<OrbitalReading>::iterator it = readings.begin();
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


  return readings;
}

// This assumes the format "prefix/YYYY-MM-DDTHH:mm:ss.sss"
OrbitalReading::timestamp_t stringToTime(std::string time_string)
{
  // find the start of the non-prefix data (right after the last '/')
  size_t prefix_end_pos = time_string.find_last_of('/');
  if (prefix_end_pos == std::string::npos)
    // Add error handling
    return 0;
  const char* read_pos = time_string.c_str() + prefix_end_pos + 1;

  struct tm t;
  // Read in the elements of the time
  t.tm_year = atoi(read_pos) - 1900;
  read_pos += 5;
  t.tm_mon = atoi(read_pos)-1;
  // Are months always 2 digits?  If not, look for "-" and skip past it
  // instead of automatically assuming a 2 digit month number.  Same applies
  // to other values.
  read_pos += 3;
  t.tm_mday = atoi(read_pos);
  read_pos += 3;
  t.tm_hour = atoi(read_pos);
  read_pos += 3;
  t.tm_min = atoi(read_pos);
  read_pos += 3;
  t.tm_sec = atoi(read_pos);
  read_pos += 3;
  int milliseconds = atoi(read_pos);

  // Convert t to a time_t
  time_t seconds = mktime(&t);
  // Multiply by 1000 to convert it to milliseconds
  OrbitalReading::timestamp_t result = seconds;
  result *= 1000;
  // add in our remaining milliseconds
  result += milliseconds;
  return result;
}
