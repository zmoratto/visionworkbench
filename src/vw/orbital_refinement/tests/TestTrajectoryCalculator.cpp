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

TEST( TrajectorCalculator, SanityCheck ) {
  EXPECT_EQ(1, 1);
}

TEST( TrajectorCalculator, CircularOrbit ) {
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
  std::size_t reading_count = (std::size_t)(period/delta_t);

  // Now re-adjust the time delta so we do an exact revolution
  delta_t = period / reading_count;

  // Start at the top of the circle,
  // moving tangent to the circle
  Vector3 p0( 0,r0, 0);
  Vector3 v0(s0, 0, 0);

  Vector3 p_next;
  Vector3 v_next;

  // Now we should move in a circle in the z=0 plane.
  // constant radius
  GravityAccelerationFunctor gravity;
  TrajectoryCalculator calc(gravity);
  for (int i = 0; i < reading_count; i++) {
    calc.calculateNextPoint(p0, v0, delta_t, p_next, v_next);

    // The radius for the new point should be about the same
    // as the starting radius.
    double r_next =
      sqrt(math::dot_prod(p_next, p_next));
    EXPECT_NEAR(r_next, r0, 1000);

    // the speed should be about the same as the starting speed
    double s_next =
      sqrt(math::dot_prod(v_next, v_next));
    EXPECT_NEAR(s_next, s0, 1);

    // The velocity should be orthogonal to the position vector.
    

    std::cout << "For iteration " << i << ":\n";
    std::cout << "  v_next is " << v_next << std::endl;
    std::cout << "  r0=" << r0 << std::endl;
    std::cout << "  r_next=" << r_next << std::endl;
    std::cout << "  r_diff = " << r_next-r0 << std::endl;
    std::cout << "  s0=" << s0 << std::endl;
    std::cout << "  s_next=" << s_next << std::endl;
    std::cout << "  s_diff = " << s_next-s0 << std::endl;
    
    p0 = p_next;
    v0 = v_next;
  }
}
