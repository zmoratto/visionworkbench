// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// TestConjugateGradient.h
#include <gtest/gtest.h>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>

TEST( TrajectorCalculator, SanityCheck )
{
  EXPECT_EQ(1, 1);
}

TEST( TrajectorCalculator, CircularOrbit )
{
    // Approximate radius of our readings.
  vw::Vector3::value_type r0 = 1860000;
  vw::Vector3::value_type GM = GravityConstants::GM_MOON;
    // speed (magnitude of velocity) at the given radius that should result in a circular orbit
  vw::Vector3::value_type s0 = sqrt(GM/r0);

    // 20 seconds, a typical time between readings.
  OrbitalReading::timestamp_t delta_t = 20000;

    // Start at the top of the circle,
    // moving tangent to the circle
  vw::Vector3 p0( 0,r0, 0);
  vw::Vector3 v0(s0, 0, 0);

  vw::Vector3 p_next;
  vw::Vector3 v_next;

    // Now we should move in a circle in the z=0 plane.
    // constant radius
  GravityAccelerationFunctor gravity;
  TrajectoryCalculator calc(gravity);
  for (int i = 0; i < 100; i++)
  {
    calc.calculateNextPoint(p0, v0, delta_t, p_next, v_next);
    
      // The radius for the new point should be about the same
      // as the starting radius.
    vw::Vector3::value_type r_next =
        sqrt(vw::math::dot_prod(p_next, p_next));
    EXPECT_NEAR(r_next, r0, 1);
    
      // the speed should be about the same as the starting speed
    vw::Vector3::value_type s_next =
        sqrt(vw::math::dot_prod(v_next, v_next));
    EXPECT_NEAR(s_next, s0, 1);
  }
}
