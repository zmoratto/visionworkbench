// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <vw/orbital_refinement/WeightCalculator.hpp>

using namespace vw;

TEST( WeightCalculator, SanityCheck ) {
  EXPECT_EQ(1, 1);
}

TEST( WeightCalculator, NoOutliers ) {

  OrbitalReading r1("", 0, 0, 0, 0);
  OrbitalReading r2("", 20000, 1000, 0, 0);
  OrbitalReading r3("", 40000, 2000, 0, 0);
  OrbitalReading r4("", 60000, 3000, 0, 0);
  OrbitalReading r5("", 80000, 4000, 0, 0);
  OrbitalReading r6("", 100000, 5000, 0, 0);
  OrbitalReading r7("", 120000, 6000, 0, 0);
  OrbitalReading r8("", 140000, 7000, 0, 0);
  OrbitalReading r9("", 160000, 8000, 0, 0);

  std::list<OrbitalReadings> readings;
  readings.push_back(r1);
  readings.push_back(r2);
  readings.push_back(r3);
  readings.push_back(r4);
  readings.push_back(r5);
  readings.push_back(r6);
  readings.push_back(r7);
  readings.push_back(r8);
  readings.push_back(r9);

  std::vector<vw::Vector3> estimated;
  std::vector<double> weights;
  estimated.resize(9);
  weights.resize(9);

  for(int k = 0; k < 9; k++) {
      vw::Vector3 temp(k*1000, 0, 0);
      estimated[k] = temp;
      weights[k] = 0.5;
  }

  WeightCalculator calc;

  calc.calculateWeights(readings, estimated, weights);

  for(int k = 0; k < 9; k++) {
      EXPECT_EQ(weights[k], 1);
  }

}
