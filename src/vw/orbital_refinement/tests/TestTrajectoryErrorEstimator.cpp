// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// TestConjugateGradient.h
#include <gtest/gtest.h>
#include <vector>
#include <vw/orbital_refinement/TrajectoryErrorEstimator.hpp>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>
#include <vw/Math/ConjugateGradient.h>

using namespace vw;
using namespace vw::math;

TEST( TrajectoryErrorEstimator, SanityCheck ) {
  EXPECT_EQ(1, 1);
}

TEST( TrajectoryErrorEstimator, CheckErrorAndGradientCalcs ) {
    // Test a straight-line trajectory with small errors in the time values.
    // We'll test a constant velocity of around (1,2,3) starting at (1001,0,0).
    // Gravity is zero.

    // Our observed coordinates are exact, but the time values are off.
  std::list<OrbitalReading> observations;
  int t0 = 0;
  observations.push_back(OrbitalReading("x1",   t0, 1001, 0, 0));
  observations.push_back(OrbitalReading("x2",  900, 1002, 2, 3));
  observations.push_back(OrbitalReading("x3", 1800, 1003, 4, 6));
  observations.push_back(OrbitalReading("x4", 3200, 1004, 6, 9));
  observations.push_back(OrbitalReading("x5", 4050, 1005, 8, 12));
  
    // We also have our starting velocity wrong.
  Vector3 p0(1001,0,0);
  Vector3 v0(.00098, .0022, .0029);

    // The locations at the observed times using p0 and v0.
    // Note that t0 is 100 according to observations, not 0.
  std::vector<Vector3> locations_at_observed_times;
  locations_at_observed_times.push_back(p0);
  locations_at_observed_times.push_back(p0 +  (900-t0)*v0);
  locations_at_observed_times.push_back(p0 + (1800-t0)*v0);
  locations_at_observed_times.push_back(p0 + (3200-t0)*v0);
  locations_at_observed_times.push_back(p0 + (4050-t0)*v0);

  
    // Create the function object we're going to optimize against.
  TrajectoryErrorEstimator error_func(observations);
  TrajectoryDecisionVariableSet initial_guess(0, p0, v0, observations);

    // See if the error and gradients are calculated as expected.
  double calculated_error = error_func(initial_guess);
  TrajectoryGradientSet gradient = error_func.gradient(initial_guess);
  
  double expected_error = 0;
  std::list<OrbitalReading>::iterator observation_iter = observations.begin();
  for (int i = 0; i < 5; ++i)
  {
    Vector3 observation = observation_iter->mCoord;
    Vector3 error = locations_at_observed_times[i] - observation;
    expected_error += dot_prod(error, error);
    
    double this_gradient = 2*dot_prod(error,v0);
    EXPECT_NEAR(gradient.t[i], this_gradient, 1e-5);
    
    ++observation_iter;
  }
  EXPECT_NEAR(calculated_error, expected_error, 1e-3);
}


TEST( TrajectoryErrorEstimator, LinearTrajectory ) {
    // Note that a lot of code here is identical to the test above.
    // Consider refactoring.
    //
    // Test a straight-line trajectory with small errors in the time values.
    // We'll test a constant velocity of around (1,2,3) starting at (1,0,0).
    // Gravity is zero.

    // Our observed coordinates are exact, but the time values are off.
  std::list<OrbitalReading> observations;
  int t0 = 0;
  observations.push_back(OrbitalReading("x1",   t0, 1001, 0, 0));
  observations.push_back(OrbitalReading("x2",  900, 1002, 2, 3));
  observations.push_back(OrbitalReading("x3", 1800, 1003, 4, 6));
  observations.push_back(OrbitalReading("x4", 3200, 1004, 6, 9));
  observations.push_back(OrbitalReading("x5", 4050, 1005, 8, 12));
  
    // We also have our starting velocity wrong.
  Vector3 p0(1001,0,0);
  Vector3 v0(.00098, .0022, .0029);

    // Create the function object we're going to optimize against.
  TrajectoryErrorEstimator error_func(observations);
  TrajectoryDecisionVariableSet initial_guess(0, p0, v0, observations);

    // Optimize
  TrajectoryDecisionVariableSet results =
      conjugate_gradient(error_func, initial_guess, ArmijoStepSize(), 500);

    // See how close we got.
  EXPECT_NEAR(results.GM, 0, 1e-3);

  std::list<OrbitalReading>::iterator observation_it = observations.begin();
  for (std::size_t i = 0; i < observations.size(); ++i)
  {
    OrbitalReading& observation = *observation_it;
    Vector3 calculated_position = results.p0 + results.v0 * (double)results.timestamps[i];
      // Expect positions within .001
    EXPECT_NEAR(calculated_position[0], observation.mCoord[0], .01);
    EXPECT_NEAR(calculated_position[1], observation.mCoord[1], .01);
    EXPECT_NEAR(calculated_position[2], observation.mCoord[2], .01);
    ++observation_it;
  }

  
}

