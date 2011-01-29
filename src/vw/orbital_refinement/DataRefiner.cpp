#include "DataRefiner.hpp"
#include "OrbitalReading.hpp"
#include <Math.h>
#include <list>

#include <vector>
#include <cmath>

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
  // Todo:  divideByOrbit(readings);

  // Next, normalize times so that each orbit starts at t=0
  normalizeReadingsByTime(readings);

  // In the matlab file:
  //  st is the data for a single orbit.  It has one row per reading,
  //  in this format:
  //    [t x y z w src(input file name) prefix(string stripped off of time stamps)]

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

  // And this:
  //   v0 = dst0(2:4)/dst0(1);
  //  dst0(1) is the average time delta
  //  dst0(2:4) is the average x, y,and z deltas
  //  so v0 is the average velocity in x, y, and z
  //  for the first 5 readings.

  //     p = [[st(1,2:4) v0]'; sM; st(:,1)];
  // st(1,2:4) is the first reading's x,y,z.
  // v0 is the x,y,z velocity estimated from first
  //   5 readings.
  // sM is a pre-defined constant, scaled Mass
  // st(:,1) is the full set of timestamps for this orbit.
  // so p is a cell with 1 column:
  //  { [averages: x,y,z,v] sM [all timestamps] }

  // dtm = average time delta between all readings in orbit
  // dts = minimum time delta between all readings in orbit
  // dt = dts/100, so it's in hundredths of a second instead of seconds

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
  std::vector<double> r(readings.size());
  calculateRadialComponents(readings, r);




  return true;
    

}