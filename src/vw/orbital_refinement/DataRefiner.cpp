#include "DataRefiner.hpp"
#include "OrbitalReading.hpp"
#include "TrajectoryGradientSet.hpp"
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <vw/orbital_refinement/TrajectoryDecisionVariableSet.hpp>
#include <vw/orbital_refinement/TrajectoryGradientSet.hpp>
#include <vw/orbital_refinement/TrajectoryErrorEstimator.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/orbital_refinement/WeightCalculator.hpp>

#include <list>

#include <vw/Math/Vector.h>
#include <vw/Math/ConjugateGradient.h>
#include <vector>
#include <cmath>
#include <limits>
// For output stream debug, remove these later
#include <cstdlib>
#include <iostream>

using namespace vw;
using namespace vw::math;

// Putting a function in an anonymous namespace makes it local to the file.
// We might consider making these private functions instead, especially if
// they use instance data, but during development this approach is often easier.
namespace
{
#if 0    
  // planetary constants
  double G=6.67259e-11;
  double M=7.36e22;
  double sGM=sqrt(M/G); // scale for GM
  double sG=G*sGM;
  double sM=M/sGM;
  double GM=G*M;
  double INF = std::numeric_limits<double>::infinity();
  double OPT_TOLERANCE = .1;
#endif

    
  void normalizeReadingsByTime(std::list<OrbitalReading>& readings)
  {
    // Sort the readings by timestamp.
    readings.sort(OrbitalReading::TimestampLess());

    // Normalize the time readings by subtracting the lowest time
    // from all of the timestamps.
    OrbitalReading::timestamp_t min_time = readings.begin()->mTime;
    for (std::list<OrbitalReading>::iterator it = readings.begin();
      it != readings.end();
      it++)
    {
      it->mTime -= min_time;
    }
  }

//   std::vector<double> calculateDifferences(std::vector<double> initial)
//   {
//       // Correctly size the vector
//       std::vector<double> diffs;
//       diffs.resize(initial.size()-1);

//       // Calculate the difference between each pair
//       for (int k =0; k < initial.size()-1; k++)
//       {
//           diffs[k] = initial[k+1]-initial[k];
//       }

//       return diffs;
//   }

//   double calculateAverage(std::vector<double> values)
//   {
//       double sum = 0;

//       // Sum the values
//       for (int k = 0; k < values.size(); k++)
//       {
//           sum += values[k];
//       }

//       // Return the average
//       return sum/values.size();
//   }

//   double getMin(std::vector<double> values)
//   {
//       std::list<double> listMin;

//       // Easiest to use a list to do this, so put the values in a list
//       for (int k = 0; k < values.size(); k++)
//       {
//           listMin.push_back(values[k]);
//       }

//       // Sort the list
//       listMin.sort();

//       // Return value of the first element
//       return *listMin.begin();
//   }

//   OrbitalReading calculateOrbitalDiffMean(std::list<OrbitalReading> orbit)
//   {
//       // Now do whatever this means:
//       //      dst0 = mean(diff(st(1:min(5,sz(1)),:)));
//       // Here's my take on it:
//       //  st is the reading array for this orbit.
//       //  sz(1) is the number of readings in this orbit.
//       //  min(5,sz(1)) is 5, or the number of readings if
//       //     there are less than 5.
//       //  st(1:min(...),:) is the first 5 rows of st.
//       //  diff(st(...)) is the difference between adjacent
//       //    rows, for those first 5 rows.
//       //  mean(diff(...)) is the average difference between
//       //    adjacent rows.

//       // Initialize vectors to the correct size for each of time,x,y,z
//       std::vector<double> times;
//       times.resize(orbit.size());
//       std::vector<double> xs;
//       xs.resize(orbit.size());
//       std::vector<double> ys;
//       ys.resize(orbit.size());
//       std::vector<double> zs;
//       zs.resize(orbit.size());

//       // Pull out the timestamp and x,y,z into their own vectors
//       int k = 0;
//       for (std::list<OrbitalReading>::iterator it = orbit.begin();
//           it != orbit.end(); it++, k++)
//       {
//           times[k] = it->mTime;
//           xs[k] = it->mCoord[0];
//           ys[k] = it->mCoord[1];
//           zs[k] = it->mCoord[2];
//       }

//       // Calculate the differences
//       std::vector<double> delta_time = calculateDifferences(times);
//       std::vector<double> delta_x = calculateDifferences(xs);
//       std::vector<double> delta_y = calculateDifferences(ys);
//       std::vector<double> delta_z = calculateDifferences(zs);

//       // Calculate the average
//       OrbitalReading delta("AverageDelta",
//               calculateAverage(delta_time),
//               calculateAverage(delta_x),
//               calculateAverage(delta_y),
//               calculateAverage(delta_z));

//       return delta;
//   }

//   OrbitalReading calculateOrbitalDiffMeanWithMin(std::list<OrbitalReading> orbit)
//   {
//       // Set the min value between 5 and the orbit size
//       int min = (5 < orbit.size())? 5 : orbit.size();

//       std::list<OrbitalReading> minReads;

//       // Grab the first min readings
//       int k = 0;
//       for (std::list<OrbitalReading>::iterator it = orbit.begin();
//           k < min; it++, k++)
//       {
//           minReads.push_back(*it);
//       }

//       // Pass the min list into the function that really does the work
//       return calculateOrbitalDiffMean(minReads);
//   }

//   OrbitalReading calculateAverageVelocity(OrbitalReading dst0)
//   {
//       // Divide each delta coordinate by the delta time
//       OrbitalReading velocity("AverageVelocity", 0,
//               dst0.mCoord[0]/dst0.mTime,
//               dst0.mCoord[1]/dst0.mTime,
//               dst0.mCoord[2]/dst0.mTime);

//       return velocity;
//   }

  Vector3 estimateInitialVelocity(const std::list<OrbitalReading>& readings)
  {
      // We base the initial velocity on the first 5 readings, or fewer if there
      // are less than 5 available.
    int reading_count = (readings.size() > 5) ? 5 : readings.size();

      // Move to the 5th reading
    std::list<OrbitalReading>::const_iterator it = readings.begin();
    std::advance(it, reading_count-1);
    Vector3 p_diff = it->mCoord - readings.begin()->mCoord;
    OrbitalReading::timestamp_t t_diff = it->mTime - readings.begin()->mTime;

      // Velocity is now just distance / time.
      // Note that you get the exact same result as if you took the
      // average distance / average time:
      //   ((x2-x1 + x1-x0)/2) / ((t2-t1 + t1-t0)/2) == (x2-x0)/(t2-t0)
    p_diff /= t_diff;
    return p_diff;
  }

#if 0
  // Calculates the convolution of two vectors, assuming that the m2 vector
  // is the smaller of the two vectors.  Pb is used to store the results of
  // the convolution
  void convolutionFunction(std::list<double>& pb, std::vector<double> m1,
          std::vector<double> m2)
  {
      // The sum is over all the values of j which lead to legal subscripts for
      // m2(j) and m1(k+1-j), specifically j = max(1,k+1-n): min(k,m).
      for (int k = 0; k < m1.size(); k++)
      {
          // Reset the sum to zero
          double temp = 0;

          for (int j = 0; j < m2.size(); j++)
          {
              if (k-j >= 0)
              {
                // Add the products over j
                //std::cout << k-j << ", " << j << std::endl;
                temp += m1[k-j]*m2[j];
              }
          }

          // Place the sum into pb, which will be of size m1+m2-1
          pb.push_back(temp);

          // Add the last piece, which is the last elements of each array at the end
          if (k == m1.size()-1)
          {
              //std::cout << k << ", " << m2.size()-1 << std::endl;
              pb.push_back( m1[k]*m2[m2.size()-1] );
          }
      }
  }

  void constructUpperBound(std::list<double>& ub, std::list<double> pb,
          std::list<double> p, double dt, double dtm)
  {
      // Add the 6 infinity elements
      for (int k = 0; k < 6; k++) {
          ub.push_back(INF);
      }

      // Add 2*sM
      ub.push_back(2*sM);

      // Add pb(2:end-1)-dt
      for (std::list<double>::iterator it = (pb.begin())++;
          it != (pb.end())--; it++)
      {
          ub.push_back(*it-dt);
      }

      // Add p(end)+dtm
      ub.push_back(p.back()+dtm);
  }

  void constructLowerBound(std::list<double>& lb, std::list<double> pb,
          std::list<double> p, double dt, double dtm)
  {
      // Add the 6 infinity elements
      for (int k = 0; k < 6; k++) {
          lb.push_back(-INF);
      }

      // Add sM
      lb.push_back(sM);

      // Add p(8)-dtm
      std::list<double>::iterator ptr = p.begin();
        // Advance the ptr 7 places
      for (int k = 0; k < 7; k++)
      {
          //std::cout << "p@" << k << ": " << *ptr << std::endl;
          ptr++;
      }
      //std::cout << "lb: ptr@8: " << *ptr-dtm << std::endl;
      lb.push_back(*ptr-dtm);

      // Add pb(2:end-1)+dt
      for (std::list<double>::iterator it = (pb.begin())++;
          it != (pb.end())--; it++)
      {
          lb.push_back(*it+dt);
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
#endif
  
}

bool OrbitalRefiner::refineOrbitalReadings(std::list<OrbitalReading>& readings)
{
  // Remember the minTime that we're about to normalize on, we have to use
  // it later
  double minTime = readings.front().mTime;

  // Next, normalize times so that it starts at t=0
  normalizeReadingsByTime(readings);

    // Get an initial velocity
  Vector3 v0 = estimateInitialVelocity(readings);

    // Data structure to hold our decision variables.
    // Initialize it with our initial guess.
  TrajectoryDecisionVariableSet decision_vars(
      GravityConstants::GM_MOON_MILLISECOND,
      readings.begin()->mCoord, v0, readings);

    // Create data structures to hold weights and estimated locations.
  std::vector<bool> weights(readings.size());
  std::vector<Vector3> estimated_locations(readings.size());

    // Calculate an initial set of locations
  GravityAccelerationFunctor gravity(decision_vars.GM);
  TrajectoryCalculator traj_calc(gravity);
  traj_calc.calculateAllPoints(decision_vars.p0, decision_vars.v0,
                               decision_vars.timestamps, estimated_locations);
  
    // Calculate an initial set of weights
  WeightCalculator weight_calc;
  weight_calc.calculateWeights(readings, estimated_locations, weights);

    // Create an error estimator
  TrajectoryErrorEstimator error_func(readings);

    // We start each iteration through the loop with a guess for our decision variables,
    // plus a set of weights for each point based on the previous guess.
  uint32 iteration_count = 0;
  while (true)
  {
    const uint32 MAX_CG_ITERATIONS = 500;
    const uint32 MAX_OUTER_ITERATIONS = 300;

      // Minimize the weighted error
    decision_vars = conjugate_gradient(error_func, decision_vars,
                                       ArmijoStepSize(), MAX_CG_ITERATIONS);

      // See if we're done
      //
      // current stopping criterion is simple iteration count
    if (++iteration_count >= MAX_OUTER_ITERATIONS)
      break;

      // If we're not done, calculate another set of weights, using the latest
      // location estimates.
    gravity.setGM(decision_vars.GM);
    traj_calc.calculateAllPoints(decision_vars.p0, decision_vars.v0,
                                 decision_vars.timestamps, estimated_locations);
    weight_calc.calculateWeights(readings, estimated_locations, weights);
  }

    // to do:  de-normalize times,
    //         do whatever needs to be done to return the data to the caller.
  
  return true;
}

#if 0
static bool oldRefineOrbitalReadings(std::list<OrbitalReading>& readings)
{
  // Remember the minTime that we're about to normalize on, we have to use
  // it later
  double minTime = readings.front().mTime;

  // Next, normalize times so that it starts at t=0
  normalizeReadingsByTime(readings);

  // In the matlab file:
  //  st is the data for a single orbit.  It has one row per reading,
  //  in this format:
  //    [t x y z w src(input file name) prefix(string stripped off of time stamps)

//   // Now we calculate the mean of differences for each orbit
//   OrbitalReading dst0 = calculateOrbitalDiffMeanWithMin(readings);

  // DEBUG FOR ORBITAL DIFF MEAN WITH MIN
  //std::cout << dst0.mId << ", " << dst0.mTime << ", " << dst0.mCoord[0]
  //        << ", " << dst0.mCoord[1] << ", " << dst0.mCoord[2] << std::endl;

  // And this:
  //   v0 = dst0(2:4)/dst0(1);
  //  dst0(1) is the average time delta
  //  dst0(2:4) is the average x, y,and z deltas
  //  so v0 is the average velocity in x, y, and z
  //  for the first 5 readings.
  Vector3 v0 = estimateInitialVelocity(dst0);

  // DEBUG FOR AVG VELOCITY
  //std::cout << v0.mId << ", " << v0.mCoord[0]
  //        << ", " << v0.mCoord[1] << ", " << v0.mCoord[2] << std::endl;


  //     p = [[st(1,2:4) v0]'; sM; st(:,1)];
  // st(1,2:4) is the first reading's x,y,z.
  // v0 is the x,y,z velocity estimated from first
  //   5 readings.
  // sM is a pre-defined constant, scaled Mass
  // st(:,1) is the full set of timestamps for this orbit.
  // so p is a cell with 1 column:
  //  { [x,y,z, average velocities] sM [all timestamps] }
//   std::list<double> p;

//   OrbitalReading first = readings.front();
//   p.push_back(first.mCoord[0]);
//   p.push_back(first.mCoord[1]);
//   p.push_back(first.mCoord[2]);
//   p.push_back(v0.mCoord[0]);
//   p.push_back(v0.mCoord[1]);
//   p.push_back(v0.mCoord[2]);
//   p.push_back(sM);

//   // Add the timestamps
//   for (std::list<OrbitalReading>::iterator it = readings.begin();
//       it != readings.end(); it++)
//   {
//       p.push_back(it->mTime);
//   }


  // dtm = average time delta between all readings in orbit
  // dts = minimum time delta between all readings in orbit
  // dt = dts/100, so it's in hundredths of a second instead of seconds
  std::vector<double> times;
  times.resize(readings.size());

  // Pull out the timestamp into its own vector
  int k = 0;
  for (std::list<OrbitalReading>::iterator it = readings.begin();
      it != readings.end(); it++, k++)
  {
      times[k] = it->mTime;
  }

/*  double dtm = calculateAverage(calculateDifferences(times));
  double dts = getMin(times);
  double dt = dts/100;

  // START OF INITIALIZATION

  // pb=conv(p(8:end),[0.5 0.5]);
  //  p(8:end) would be all timestamps
  //  Convolution is a mathematical operation on two functions f and g,
  //  producing a third function that is typically viewed as a modified
  //  version of one of the original functions
  std::list<double> pb;
  std::vector<double> halfMatrix(2, 0.5);
  convolutionFunction(pb, times, halfMatrix);

  // DEBUG FOR CONVOLUTION
  /*int j = 1;
  std::cout << "Orbit size: " << orbit.size() << std::endl;
  for (std::list<double>::iterator it = pb.begin();
      it != pb.end(); it++, j++)
  {
      std::cout << j << ", " << *it << std::endl;
  }

  // ub= [ inf*ones(6,1); 2*sM; pb(2:end-1)-dt; p(end)+dtm];
  //   upper bound (of???)
  std::list<double> ub;
  constructUpperBound(ub, pb, p, dt, dtm);

  // lb= [-inf*ones(6,1);   sM;   p(8)-dtm; pb(2:end-1)+dt];
  //   lower bound (of???)
  std::list<double> lb;
  constructLowerBound(lb, pb, p, dt, dtm); */

  //  [p,resnorm,residual,exitflag,output,lambda]=lsqnonlin(@(p)mbrOrbRefOi(p,st,sG),p,lb,ub,options);
  //  This really boils down to calculating the acceleration of each coordinate

  //  First grab all the positions and corresponding velocities out of p
  vw::Vector3 position;
  position.set_size(3);

  vw::Vector3 velocity;
  velocity.set_size(3);

  std::list<double>::iterator it = p.begin();
  // The first 3 are position values
  for (int k = 0; k < 3; k++, it++) {
      position[k] = *it;
  }
  // The next 3 are velocity values
  for (int k = 0; k < 3; k++, it++) {
      velocity[k] = *it;
  }

  // Initialize all the components
  TrajectoryDecisionVariableSet tempResult(GM, position, velocity, readings);

  TrajectoryErrorEstimator errorEst(readings);

  double lastError = 0;
  double error = 0;
  double deltaError = 0;

  // For loop to optimize orbit

  do {
      // Create the variable set for the current values
      TrajectoryDecisionVariableSet tempVars(tempResult.GM, tempResult.p0, tempResult.v0, readings);

      // Remember the previous error
      lastError = error;
      // Calculate the error for the new variable set
      error = errorEst(tempVars);
      // Calculate the delta
      deltaError = error - lastError;

      // Call conjugate_gradient to get the next set of values for optimizing
      tempResult = conjugate_gradient(errorEst, tempVars, ArmijoStepSize(), 500);
      
  } while(deltaError > OPT_TOLERANCE);
  

/*
  // Calculate the radial component of each reading.
  // It's in the same order as 'readings'
  std::vector<double> r(orbit.size());
  calculateRadialComponents(orbit, r);*/

  return true;
    

}

#endif
