// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <vw/orbital_refinement/WeightCalculator.hpp>

using namespace vw;

TEST( WeightCalculator, SanityCheck )
{
  EXPECT_EQ(1, 1);
}

TEST( WeightCalculator, NoOutliers )
{
  std::list<OrbitalReading> readings;
  std::vector<vw::Vector3> estimated;
  std::vector<double> weights;
  estimated.resize(9);
  weights.resize(9);

  // Setup the data, time increments of 20000ms, x increments of 1000
  for(int k = 0; k < 9; k++)
  {
      vw::Vector3 temp(k*1000, 0, 0);
      OrbitalReading r_temp("", k*20000, k*1000, 0, 0);

      estimated[k] = temp;
      weights[k] = 0.5;
      readings.push_back(r_temp);
  }

  // initialize and call weight calculation
  WeightCalculator calc;

  calc.calculateWeights(readings, estimated, weights);

  // make sure all the weights are one
  for(int k = 0; k < 9; k++)
  {
    EXPECT_EQ(weights[k], 1);
  }
}
