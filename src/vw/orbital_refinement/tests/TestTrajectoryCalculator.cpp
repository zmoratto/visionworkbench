// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// TestConjugateGradient.h
#include <gtest/gtest.h>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>

using namespace vw;

TEST( TrajectoryCalculator, SanityCheck ) {
  EXPECT_EQ(1, 1);
}

TEST( TrajectoryCalculator, CircularOrbit ) {
  // Approximate radius of our readings (meters).
  double r0 = 1860000;
  double GM = GravityConstants::GM_MOON_MILLISECOND;
  // speed (magnitude of velocity) at the given radius that
  // should result in a circular orbit (m/millisecond)
  double s0 = sqrt(GM/r0);
  const double PI = acos((long double)-1);
  // Total time expected to do a full revolution (in milliseconds)
  OrbitalReading::timestamp_t period = 2*PI*r0/s0;

  // 20 seconds, a typical time between readings.
  OrbitalReading::timestamp_t delta_t = 20000;

  // Number of readings to do a full revolution
  int reading_count = (int)(period/delta_t);

  // Now re-adjust the time delta so we do an exact revolution
  delta_t = period / reading_count;

  // Start at the top of the circle,
  // moving tangent to the circle
  Vector3 p0( 0,r0, 0);
  Vector3 v0(s0, 0, 0);

  Vector3 p_next;
  Vector3 v_next;

  // Print out initial position and velocity:
//   std::cout << "\nInitial Position: " << p0 << std::endl;
//   std::cout << "Initial Velocity: " << v0 << std::endl;

  // Now we should move in a circle in the z=0 plane.
  // constant radius
  GravityAccelerationFunctor gravity(GM);
  TrajectoryCalculator calc(gravity);
  for (int i = 0; i < reading_count; i++) {
    calc.calculateNextPoint(p0, v0, delta_t, p_next, v_next);

    // The radius for the new point should be about the same
    // as the starting radius.
    double r_next =
      sqrt(math::dot_prod(p_next, p_next));
    EXPECT_NEAR(r_next, r0, 100);

    // the speed should be about the same as the starting speed
    double s_next =
      sqrt(math::dot_prod(v_next, v_next));
    EXPECT_NEAR(s_next, s0, 1);

    // The position should be in the z=0 plane
    EXPECT_EQ(p_next[2], 0);

    // The position should be the appropriate proportion around the circle
    double angle = PI/2 - 2*PI*(i+1)/reading_count;
    EXPECT_NEAR(p_next[0]/r_next, cos(angle), .01);
    EXPECT_NEAR(p_next[1]/r_next, sin(angle), .01);

//     std::cout << "For iteration " << i << ":\n";
//     std::cout << "  v_next is " << v_next << std::endl;
//     std::cout << "  r0=" << r0 << std::endl;
//     std::cout << "  r_next=" << r_next << std::endl;
//     std::cout << "  r_diff = " << r_next-r0 << std::endl;
//     std::cout << "  s0=" << s0 << std::endl;
//     std::cout << "  s_next=" << s_next << std::endl;
//     std::cout << "  s_diff = " << s_next-s0 << std::endl;
    
    p0 = p_next;
    v0 = v_next;
  }

  // print out the final positions.
//   std::cout << "\nFinal Position: " << p0 << std::endl;
//   std::cout << "Final Velocity: " << v0 << std::endl;
}

TEST( TrajectoryCalculator, StraightLine ) {
  GravityAccelerationFunctor gravity(0);
  TrajectoryCalculator traj_calc(gravity);
  
  Vector3 p0(1.0, 0.0, 0.0);
  Vector3 v0(.001, .002, .003);
  
  Vector3 cur_p = p0;
  Vector3 cur_v = v0;
  for (int i = 1; i < 5; i++)
  {
    Vector3 next_p;
    Vector3 next_v;
    traj_calc.calculateNextPoint(cur_p, cur_v, 1000, next_p, next_v);
    ASSERT_NEAR(next_p[0], (double)i + 1, 1e-5);
    ASSERT_NEAR(next_p[1], i*2, 1e-5);
    ASSERT_NEAR(next_p[2], i*3, 1e-5);
    
    ASSERT_NEAR(next_v[0], v0[0], 1e-6);
    ASSERT_NEAR(next_v[1], v0[1], 1e-6);
    ASSERT_NEAR(next_v[2], v0[2], 1e-6);
    
    cur_p = next_p;
    cur_v = next_v;
  }
}

TEST( TrajectoryCalculator, AllPointsAtOnce ) {
    // This is just like the circle test, but you
    // get all points all at once instead of one at a time.
  
  // Approximate radius of our readings (meters).
  double r0 = 1860000;
  double GM = GravityConstants::GM_MOON_MILLISECOND;
  // speed (magnitude of velocity) at the given radius that
  // should result in a circular orbit (m/millisecond)
  double s0 = sqrt(GM/r0);
  const double PI = acos((long double)-1);
  // Total time expected to do a full revolution (in milliseconds)
  OrbitalReading::timestamp_t period = 2*PI*r0/s0;

  // 20 seconds, a typical time between readings.
  OrbitalReading::timestamp_t delta_t = 20000;

  // Number of readings to do a full revolution
  int reading_count = (int)(period/delta_t);

  // Now re-adjust the time delta so we do an exact revolution
  delta_t = period / reading_count;

    // Fill in a vector of time values
  std::vector<OrbitalReading::timestamp_t> times(reading_count);
  for (int i = 0; i < reading_count; ++i)
    times[i] = delta_t * i;

  // Start at the top of the circle,
  // moving tangent to the circle
  Vector3 p0( 0,r0, 0);
  Vector3 v0(s0, 0, 0);

    // Create a place to store the results
  std::vector<vw::Vector3> estimated_locations(reading_count);

    // Get all the points
  GravityAccelerationFunctor gravity(GM);
  TrajectoryCalculator calc(gravity);
  calc.calculateAllPoints(p0, v0, times, estimated_locations);

  // Check if we moved in a circle in the z=0 plane.
  // constant radius
  for (int i = 0; i < reading_count; i++)
  {
    // The radius for the new point should be about the same
    // as the starting radius.
    double r = norm_2(estimated_locations[i]);
    EXPECT_NEAR(r, r0, 100);

    // The position should be in the z=0 plane
    EXPECT_EQ(estimated_locations[i][2], 0);

    // The position should be the appropriate proportion around the circle
    double angle = PI/2 - 2*PI*(i)/reading_count;
    EXPECT_NEAR(estimated_locations[i][0]/r, cos(angle), .01);
    EXPECT_NEAR(estimated_locations[i][1]/r, sin(angle), .01);
  }
}
