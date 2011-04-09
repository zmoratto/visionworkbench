#include <ORBARefiner.hpp>
#include <vw/orbital_refinement/DataRefiner.hpp>
#include <vw/orbital_refinement/OrbitalReading.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <vw/orbital_refinement/WeightCalculator.hpp>
#include <OrbitalCameraReading.hpp>
#include <ObservationSet.hpp>

#include <list>
#include <vector>

#include "ORBADecisionVariableSet.hpp"

using namespace vw::ba;

namespace vw{
namespace ORBA{


    void normalizeReadingsByTime(std::vector<OrbitalCameraReading>& readings)
    {
        // Sort the readings by timestamp.
        readings.sort(OrbitalReading::TimestampLess());

        // Normalize the time readings by subtracting the lowest time
        // from all of the timestamps.
        OrbitalReading::timestamp_t min_time = readings.begin()->mTime;
        for (std::vector<OrbitalCameraReading>::iterator it = readings.begin();
          it != readings.end();
          it++)
        {
          it->mTime -= min_time;
        }
    }

    void denormalizeReadingsByTime(std::vector<OrbitalCameraReading>& readings,
                                 OrbitalReading::timestamp_t base_time)
    {
      // denormalize the time readings by adding the base time
      // to all of the timestamps.
      for (std::vector<OrbitalCameraReading>::iterator it = readings.begin();
        it != readings.end();
        it++)
      {
        it->mTime += base_time;
      }
    }

    Vector3 estimateInitialVelocity(const std::vector<OrbitalCameraReading>& readings)
    {
          // We base the initial velocity on the first 5 readings, or fewer if there
          // are less than 5 available.
        int reading_count = (readings.size() > 5) ? 5 : readings.size();

          // Move to the 5th reading
        std::vector<OrbitalCameraReading>::const_iterator it = readings.begin();
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

    bool refineORBAReadings(ObservationSet& obs, const Vector3& sigma_p,
            const Vector3& sigma_r, const Vector3& sigma_s, double sigma_t)
    {
        // Extract the data from the observation set into a data format to
        // pass into the OrbitalRefiner
        
        std::vector<OrbitalCameraReading> readings = obs.getReadings();
        std::list<OrbitalReading> orReadings;
        std::list<OrbitalReading> refinedOrReadings;

        for( std::vector<OrbitalCameraReading>::iterator it = readings.begin();
                it != readings.end();
                it++) 
        {
            orReadings.push_back(new OrbitalReading(it->mId, it->mTime, it->mCoord));
        }

        OrbitalRefiner orRefiner = new OrbitalRefiner();
        if( !orRefiner.refineOrbitalReadings(orReadings, refinedOrReadings) )
        {
            std::cerr << "Something went wrong when adjusting the orbital readings" << std::endl;
            return false;
        }

        // OR has been called, now put the refined data back into the
        // observation set
        int i = 0;
        for( std::list<OrbitalReading>::iterator it = refinedOrReadings.begin();
                it != refinedOrReadings.end();
                it++, i++)
        {
            OrbitalCameraReading temp = obs.getReading(i);
            // Make sure the IDs match, this should work as long as the
            // observations have been sorted by time when passed into this method
            if (temp.mId == it->mId)
            {
                temp.mCoord = it->mCoord;
                temp.mTime = it->mTime;
            }
        }

        // Reset the readings
        readings = obs.getReadings();

        // Remember the time we're normalizing on, then normalize
        OrbitalReading::timestamp_t min_time = readings.front().mTime;
        normalizeReadingsByTime(readings);

        // Get an initial velocity
        Vector3 v0 = estimateInitialVelocity(readings);

        // Data structure to hold our decision variables.
        // Initialize it with our initial guess.
        ORBADecisionVariableSet decision_vars(
          GravityConstants::GM_MOON_MILLISECOND,
          readings.begin()->mCoord, v0, readings, obs.getControlNetwork(),
          sigma_p, sigma_r, sigma_s, sigma_t);

        // Create data structures to hold weights and estimated locations.
        // Weights are all initialized to 0.5
        std::vector<double> weights(readings.size(), 0.5);
        std::vector<Vector3> estimated_locations(readings.size());

        // Calculate an initial set of locations
        GravityAccelerationFunctor gravity(decision_vars.GM);
        TrajectoryCalculator traj_calc(gravity);

        // Calculate an initial set of weights
        WeightCalculator weight_calc;

        ORBAErrorEstimator error_func(readings, weights);

        // We start each iteration through the loop with a guess for our decision variables,
        // plus a set of weights for each point based on the previous guess.
        uint32 iteration_count = 0;

        const double IMPROVEMENT_THRESH = 1e-10;
        const uint32 IMPROVEMENT_THRESH_TOL = 5;
        double prev_error = 0;
        uint32 thresh_ctr = 0;
        while (true)
        {
            const uint32 MAX_CG_ITERATIONS = 500;
            const uint32 MAX_OUTER_ITERATIONS = 1000;

            prev_error = error_func(decision_vars);
              // Minimize the weighted error
            decision_vars = conjugate_gradient(error_func, decision_vars,
                                               ArmijoStepSize(), MAX_CG_ITERATIONS);

            double this_error = error_func(decision_vars);
            double delta = prev_error - this_error;

            if (delta < IMPROVEMENT_THRESH)
            {
                thresh_ctr++;
            } else {
                thresh_ctr = 0;
            }
              // See if we're done
            if (thresh_ctr >= IMPROVEMENT_THRESH_TOL ||
                ++iteration_count >= MAX_OUTER_ITERATIONS)
            {
              break;
            }

              // If we're not done, calculate another set of weights, using the latest
              // location estimates.
            gravity.setGM(decision_vars.GM);
            traj_calc.calculateAllPoints(decision_vars.p0, decision_vars.v0,
                                         decision_vars.timestamps, estimated_locations);
            weight_calc.calculateWeights(readings, estimated_locations, weights);
        }

        // Place the adjusted times and coordinates into the passed in list of readings.
        // They are already in estimated_locations and decision_vars.timestamps.
        // We just need to transfer them into the readings list.
        std::vector<OrbitalCameraReading>::iterator reading_it = readings.begin();
        for (int i = 0;
             reading_it != readings.end();
             ++reading_it, ++i)
        {
          reading_it->mTime = decision_vars.timestamps[i];
          reading_it->mCoord = estimated_locations[i];
        }

        // Next, denormalize times
        denormalizeReadingsByTime(readings, min_time);
    }

}} // vw::ORBA

