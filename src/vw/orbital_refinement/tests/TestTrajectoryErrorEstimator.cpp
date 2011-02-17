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

TEST( TrajectoryErrorEstimator, LinearTrajectory ) {
    // Test a straight-line trajectory with small errors in the time values.
    // We'll test a constant velocity of (1,2,3) starting at (1,0,0).
    // Gravity is zero.
    //
    // Correct points [t: x y z], with t in milliseconds, are:
    //   [0 1 0 0]
    //   [1000 2 2 3]
    //   [2000 3 4 6]
    //   [3000 4 6 9]
    //   [4000 5 8 12]
  std::vector<OrbitalReading> correct_points;
    // "ID", t, x, y, z
  correct_points.push_back(OrbitalReading("p1",    0, 1, 0,  0));
  correct_points.push_back(OrbitalReading("p2", 1000, 2, 2,  3));
  correct_points.push_back(OrbitalReading("p3", 2000, 3, 4,  6));
  correct_points.push_back(OrbitalReading("p4", 3000, 4, 6,  9));
  correct_points.push_back(OrbitalReading("p5", 4000, 5, 8, 12));
  Vector3 correct_p0 = correct_points[0].mCoord;
  Vector3 correct_v0(.001, .002, .003);

    // Our observed coordinates are exact, but the time values are off.
  std::list<OrbitalReading> observations;
  observations.push_back(OrbitalReading("x1",  100, 1, 0, 0));
  observations.push_back(OrbitalReading("x2",  900, 2, 2, 3));
  observations.push_back(OrbitalReading("x3", 1800, 3, 4, 6));
  observations.push_back(OrbitalReading("x4", 3200, 4, 6, 9));
  observations.push_back(OrbitalReading("x5", 4050, 5, 8, 12));
  
    // We also have our starting position and velocity wrong.
  Vector3 p0(.8, .2, .1);
  Vector3 v0(.00098, .0022, .0029);

    // Create the function object we're going to optimize against.
  TrajectoryErrorEstimator error_func(observations);
  TrajectoryDecisionVariableSet initial_guess(0, p0, v0, observations);
  
    // Optimize
  TrajectoryDecisionVariableSet results =
      conjugate_gradient(error_func, initial_guess, ArmijoStepSize(), 500);

    // See how close we got.
  EXPECT_NEAR(results.GM, 0, 1e-3);
  EXPECT_NEAR(results.p0[0], correct_p0[0], 1e-3);
  EXPECT_NEAR(results.p0[1], correct_p0[1], 1e-3);
  EXPECT_NEAR(results.p0[2], correct_p0[2], 1e-3);
  EXPECT_NEAR(results.v0[0], correct_v0[0], 1e-3);
  EXPECT_NEAR(results.v0[1], correct_v0[1], 1e-3);
  EXPECT_NEAR(results.v0[2], correct_v0[2], 1e-3);
  
  for (std::size_t i = 0; i < correct_points.size(); ++i)
  {
      // Expect times within 2 milliseconds.
    EXPECT_NEAR(results.timestamps[i], correct_points[i].mTime, 2);
  }
}

