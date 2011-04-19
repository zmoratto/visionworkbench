#include <vw/ORBA/ORBARefiner.hpp>
#include <vw/ORBA/ORBAErrorEstimator.hpp>
#include <vw/orbital_refinement/DataRefiner.hpp>
#include <vw/orbital_refinement/OrbitalReading.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <vw/orbital_refinement/WeightCalculator.hpp>
#include <vw/ORBA/OrbitalCameraReading.hpp>
#include <vw/ORBA/ObservationSet.hpp>
#include <vw/ORBA/ORBADecisionVariableSet.hpp>
#include <vw/Math/ConjugateGradient.h>

#include <list>
#include <vector>

#include "ObservationSet.hpp"

using namespace vw::ba;

namespace vw{
namespace ORBA{


// Anonymous namespace for local utilties
namespace 
{
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
}
    
    bool ORBARefiner::refineORBAReadings(
        ObservationSet obs, ObservationSet& dest, const Vector2& sigma_p,
        const Vector3& sigma_r, const Vector3& sigma_s, double sigma_t)
    {
        // First, get orbital_refinement to make a first
        // pass at fixing the orbit.

        // Normalize the times.  Do this up front so we're sure we aren't getting
        // any conflicting normalizations
        obs.normalizeTimes();
      
        // Extract the data from the observation set into a data format to
        // pass into the OrbitalRefiner
        const std::vector<OrbitalCameraReading>& readings = obs.getReadings();
        std::list<OrbitalReading> orReadings;
        std::list<OrbitalReading> refinedOrReadings;
        for(std::vector<OrbitalCameraReading>::const_iterator it = readings.begin();
            it != readings.end();
            it++) 
        {
          orReadings.push_back(OrbitalReading(it->mId, it->mTime, it->mCoord));
        }

        OrbitalRefiner orRefiner;
        if( !orRefiner.refineOrbitalReadings(orReadings, refinedOrReadings) )
        {
            std::cerr << "Something went wrong when adjusting the orbital readings" << std::endl;
            return false;
        }

        // Data structure to hold our decision variables.
        // Initialize it with our initial guess.
        ORBADecisionVariableSet decision_vars(
            orRefiner.getCalculatedGM(),
            orRefiner.getCalculatedInitialPosition(),
            orRefiner.getCalculatedInitialVelocity(),
            refinedOrReadings, obs.getControlNetwork(),
            sigma_p, sigma_r, sigma_s, sigma_t);

        // Create data structures to hold weights and estimated locations.
        // Weights are all initialized to 0.5
        std::vector<double>& weights = orRefiner.getInlierWeights();
        std::vector<Vector3> estimated_locations(readings.size());

        // Create calculator/helper objects
        GravityAccelerationFunctor gravity(decision_vars.GM);
        TrajectoryCalculator traj_calc(gravity);
        WeightCalculator weight_calc;
        ORBAErrorEstimator error_func(boost::shared_ptr<ObservationSet>(&obs), weights);

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


              // Store the calculated locations.
              // We do this even if this is the final iteration, as the locations will stored
              // in the output.
            gravity.setGM(decision_vars.GM);
            traj_calc.calculateAllPoints(decision_vars.p0, decision_vars.v0,
                                         decision_vars.timestamps, estimated_locations);
            
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
            
              // Calculate weights to be used in next iteration, using the latest location estimates.
            weight_calc.calculateWeights(readings, estimated_locations, weights);
        }

        // Place the adjusted times, coordinates, and camera orientations
        // into the passed in observation set. Here's where we get our data:
        // * Changes to ControlPoint locations are already stored in the ControlNetwork;
        //   they were modified directly during optimization.
        // * The times are stored as decision variables
        // * OR-calculated coords are stored in estimated_locations.
        // * Camera coords are stored as decision variables, as offsets from our observations.
        // * Camera orientations are stored as decision variables, also as offsets.
        //
        // Here's where that data will be stored:
        // * ControlPoint locations are stored in dest's ControlNetwork
        // * Camera position and orientation are stored in each reading's camera object
        // * OR time and position are stored in the reading outside of the camera
        dest.setControlNetwork(obs.getControlNetwork());
        dest.setExpectedReadingCount(readings.size());
        int i = 0;
        BOOST_FOREACH(const OrbitalCameraReading& original_reading, obs.getReadings())
        {
            // Create the reading, store original data and revised time.
          OrbitalCameraReading reading(original_reading.mId,
                                       decision_vars.timestamps[i],
                                       original_reading.mCamera,
                                       original_reading.mImageID);
            // Add the camera offsets
          AdjustedCameraModel adj_cam(reading.mCamera, decision_vars.pj[i],
                                      Quaternion<double>(decision_vars.cj_second[i]));
          reading.mCamera->set_camera_center(adj_cam.camera_center(Vector2(0,0)));
          reading.mCamera->set_camera_pose(adj_cam.camera_pose(Vector2(0,0)));

            // Store the OR-calculated location
          reading.mCoord = estimated_locations[i];
          
          dest.addReading(reading);
          i++;
        }

        // Denormalize times
        dest.setTimeNormalizationBase(obs.getTimeNormalizationBase());
        dest.denormalizeReadingTimes();

        return true;
    }

}} // vw::ORBA

