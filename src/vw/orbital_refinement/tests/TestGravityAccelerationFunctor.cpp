// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// TestConjugateGradient.h
#include <gtest/gtest.h>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>

using namespace vw;
using namespace math;

TEST( GravityAcclerationFunctor, SanityCheck ) {
  EXPECT_EQ(1, 1);
}

TEST( GravityAcclerationFunctor, OnAxisMoonGravity ) {
  GravityAccelerationFunctor gravity(GravityConstants::GM_MOON);

    // The magnitude of acceleration at (0,1860000,0) meters should be GM/r^2.
    // It should be in the -y direction.
  Vector3::value_type r = 1860000;
  Vector3 position(0, r, 0);
  Vector3 acceleration = gravity(position);

  EXPECT_EQ(acceleration[0], 0);
  EXPECT_EQ(acceleration[1], -GravityConstants::GM_MOON/(r*r));
  EXPECT_EQ(acceleration[2], 0);
}

TEST( GravityAcclerationFunctor, OffAxisMoonGravity ) {
  GravityAccelerationFunctor gravity(GravityConstants::GM_MOON);

    // Pick position based on three off-axis points
  Vector3 position(-372000, 999999.2, 314.15);
  Vector3::value_type r = sqrt(dot_prod(position, position));
  Vector3 acceleration = gravity(position);

    // Make sure the magnitude of the acceleration matches
  Vector3::value_type accel_mag = sqrt(dot_prod(acceleration, acceleration));
  EXPECT_NEAR(accel_mag, GravityConstants::GM_MOON/(r*r), .1);
}

