#include "DataRefiner.hpp"
#include "OrbitalReading.hpp"
#include <Math.h>
#include <list>

#include <vector>
#include <cmath>
// For output stream debug, remove these later
#include <cstdlib>
#include <iostream>

// Putting a function in an anonymous namespace makes it local to the file.
// We might consider making these private functions instead, especially if
// they use instance data, but during development this approach is often easier.
namespace
{
  // planetary constants
  double G=6.67259e-11;
  double M=7.36e22;
  double sGM=sqrt(M/G); // scale for GM
  double sG=G*sGM;
  double sM=M/sGM;
  double GM=G*M;


  void divideByOrbit(std::list< std::list<OrbitalReading> >& orbits,
          std::list<OrbitalReading> readings)
  {
      std::list<OrbitalReading> tempOrbit;
      
      // Iterate through all the readings
      for (std::list<OrbitalReading>::iterator it = readings.begin();
            it != readings.end(); ++it)
      {
          // If the tempOrbit is empty, add the reading we're on
          if (tempOrbit.empty())
              tempOrbit.push_back(*it);
          else
          {
              // Otherwise, check if the difference between the current reading
              // and the first reading in the orbit is within the 3300 second range
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
                  std::cout << "clearing orbit";
                  tempOrbit.clear();
                  tempOrbit.push_back(*it);
              }
          }

      }
  }

  void normalizeReadingsByTime(std::list<OrbitalReading>& readings)
  {
    // Sort the readings by timestamp.
    readings.sort(OrbitalReading::TimestampLess());

    // Normalize the time readings by subtracting the lowest time
    // from all of the timestamps.
    OrbitalReading::timestamp_t min_time = readings.begin()->mTime;
    for (std::list<OrbitalReading>::iterator it = readings.begin();
      it != readings.end();
      ++it)
    {
      it->mTime -= min_time;
    }
  }

  std::vector<double> calculateDifferences(std::vector<double> initial)
  {
      // Correctly size the vector
      std::vector<double> diffs;
      diffs.resize(initial.size()-1);

      // Calculate the difference between each pair
      for (int k =0; k < initial.size()-1; ++k)
      {
          diffs[k] = initial[k+1]-initial[k];
      }

      return diffs;
  }

  double calculateAverage(std::vector<double> values)
  {
      double sum = 0;

      // Sum the values
      for (int k = 0; k < values.size(); ++k)
      {
          sum += values[k];
      }

      // Return the average
      return sum/values.size();
  }

  double getMin(std::vector<double> values)
  {
      std::list<double> listMin;

      // Easiest to use a list to do this, so put the values in a list
      for (int k = 0; k < values.size(); ++k)
      {
          listMin.push_back(values[k]);
      }

      // Sort the list
      listMin.sort();

      // Return value of the first element
      return *listMin.begin();
  }

  OrbitalReading calculateOrbitalDiffMean(std::list<OrbitalReading> orbit)
  {
      // Now do whatever this means:
      //      dst0 = mean(diff(st(1:min(5,sz(1)),:)));
      // Here's my take on it:
      //  st is the reading array for this orbit.
      //  sz(1) is the number of readings in this orbit.
      //  min(5,sz(1)) is 5, or the number of readings if
      //     there are less than 5.
      //  st(1:min(...),:) is the first 5 rows of st.
      //  diff(st(...)) is the difference between adjacent
      //    rows, for those first 5 rows.
      //  mean(diff(...)) is the average difference between
      //    adjacent rows.

      // Initialize vectors to the correct size for each of time,x,y,z
      std::vector<double> times;
      times.resize(orbit.size());
      std::vector<double> xs;
      xs.resize(orbit.size());
      std::vector<double> ys;
      ys.resize(orbit.size());
      std::vector<double> zs;
      zs.resize(orbit.size());

      // Pull out the timestamp and x,y,z into their own vectors
      int k = 0;
      for (std::list<OrbitalReading>::iterator it = orbit.begin();
          it != orbit.end(); ++it, ++k)
      {
          times[k] = it->mTime;
          xs[k] = it->mCoord[0];
          ys[k] = it->mCoord[1];
          zs[k] = it->mCoord[2];
      }

      // Calculate the differences
      std::vector<double> delta_time = calculateDifferences(times);
      std::vector<double> delta_x = calculateDifferences(xs);
      std::vector<double> delta_y = calculateDifferences(ys);
      std::vector<double> delta_z = calculateDifferences(zs);

      // Calculate the average
      OrbitalReading delta("AverageDelta",
              calculateAverage(delta_time),
              calculateAverage(delta_x),
              calculateAverage(delta_y),
              calculateAverage(delta_z));

      return delta;
  }

  OrbitalReading calculateOrbitalDiffMeanWithMin(std::list<OrbitalReading> orbit)
  {
      // Set the min value between 5 and the orbit size
      int min = (5 < orbit.size())? 5 : orbit.size();

      std::list<OrbitalReading> minReads;

      // Grab the first min readings
      int k = 0;
      for (std::list<OrbitalReading>::iterator it = orbit.begin();
          k < min; ++it, ++k)
      {
          minReads.push_back(*it);
      }

      // Pass the min list into the function that really does the work
      return calculateOrbitalDiffMean(minReads);
  }

  OrbitalReading calculateAverageVelocity(OrbitalReading dst0)
  {
      // Divide each delta coordinate by the delta time
      OrbitalReading velocity("AverageVelocity", 0,
              dst0.mCoord[0]/dst0.mTime,
              dst0.mCoord[1]/dst0.mTime,
              dst0.mCoord[2]/dst0.mTime);

      return velocity;
  }

  void calculateRadialComponents(std::list<OrbitalReading>& readings, std::vector<double>& r)
  {
    std::vector<double>::iterator r_it = r.begin();
    for (std::list<OrbitalReading>::iterator reading_it = readings.begin();
         reading_it != readings.end();
         ++reading_it, ++r_it)
    {
      OrbitalReading& reading = *reading_it;
      // There's probably a function for this in vw.
      double r = sqrt(reading.mCoord[0]*reading.mCoord[0] +
                      reading.mCoord[1]*reading.mCoord[1] +
                      reading.mCoord[2]*reading.mCoord[2]);
      *r_it = r;
    }
  }

  
}

bool OrbitalRefiner::refineOrbitalReadings(std::list<OrbitalReading>& readings)
{
  // divide readings into orbits
  // This is done by finding time gaps of sufficient size.
  // Interesting to me that in the matlab code, gaps are found
  // *before* sorting by time...isn't this a bug?
  std::list< std::list<OrbitalReading> > orbits;
  divideByOrbit(orbits, readings);

  std::cout << "Number of orbits: " << orbits.size();
  /*for (std::list< std::list<OrbitalReading> >::iterator read_orb = orbits.begin();
          read_orb != orbits.end(); ++read_orb)
  {
      std::list<OrbitalReading> orbit = read_orb;

      // remember the minTime that we're about to normalize on, we have to add
      // it back

      // Next, normalize times so that each orbit starts at t=0
      normalizeReadingsByTime(orbit);

      // In the matlab file:
      //  st is the data for a single orbit.  It has one row per reading,
      //  in this format:
      //    [t x y z w src(input file name) prefix(string stripped off of time stamps)

      // Now we calculate the mean of differences for each orbit
      OrbitalReading dst0 = calculateOrbitalDiffMeanWithMin(orbit);

      // And this:
      //   v0 = dst0(2:4)/dst0(1);
      //  dst0(1) is the average time delta
      //  dst0(2:4) is the average x, y,and z deltas
      //  so v0 is the average velocity in x, y, and z
      //  for the first 5 readings.
      OrbitalReading v0 = calculateAverageVelocity(dst0);

      //     p = [[st(1,2:4) v0]'; sM; st(:,1)];
      // st(1,2:4) is the first reading's x,y,z.
      // v0 is the x,y,z velocity estimated from first
      //   5 readings.
      // sM is a pre-defined constant, scaled Mass
      // st(:,1) is the full set of timestamps for this orbit.
      // so p is a cell with 1 column:
      //  { [averages: x,y,z,v] sM [all timestamps] }
      std::list<double> p;

      OrbitalReading first = orbit.front();
      p.push_back(first->mCoord[0]);
      p.push_back(first->mCoord[1]);
      p.push_back(first->mCoord[2]);
      p.push_back(v0);
      p.push_back(sM);

      // Add the timestamps
      for (std::list<OrbitalReading>::iterator it = orbit.begin();
          it != orbit.end(); ++it)
      {
          p.push_back(it->mTime);
      }

      
      // dtm = average time delta between all readings in orbit
      // dts = minimum time delta between all readings in orbit
      // dt = dts/100, so it's in hundredths of a second instead of seconds
      std::vector<double> times;
      times.resize(orbit.size());

      // Pull out the timestamp into its own vector
      int k = 0;
      for (std::list<OrbitalReading>::iterator it = orbit.begin();
          it != orbit.end(); ++it, ++k)
      {
          times[k] = it->mTime;
      }

      double dtm = calculateAverage(calculateDifferences(times));
      double dts = getMin(times);
      double dt = dts/100;

      // pb=conv(p(8:end),[0.5 0.5]);
      //  p(8:end) would be all timestamps starting with the 3rd reading.
      //  I'm not sure what the point is here...not familiar with the math

      // ub= [ inf*ones(6,1); 2*sM; pb(2:end-1)-dt; p(end)+dtm];
      //   upper bound (of???)

      // lb= [-inf*ones(6,1);   sM;   p(8)-dtm; pb(2:end-1)+dt];
      //   lower bound (of???)

      //  [p,resnorm,residual,exitflag,output,lambda]=lsqnonlin(@(p)mbrOrbRefOi(p,st,sG),p,lb,ub,options);
      //  Solves a non-linear least squares problem.
      // I think this is where CG comes in?  I think this least squares method is minimizing a function, which is
      // what CG would do, but using a different method?



      // Calculate the radial component of each reading.
      // It's in the same order as 'readings'
      std::vector<double> r(orbit.size());
      calculateRadialComponents(orbit, r);
  }*/



  return true;
    

}