// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <vw/orbital_refinement/WeightCalculator.hpp>

using namespace vw;

double tmp[][4] = {{ -1776242.49767, 46779.2258986, -472196.070251 , 200 },
                  { -1776418.70377, 87119.7764482, -462209.304457  , 450 },
                  { -1775876.75727, 128398.562437, -452410.722776  , 620 },
                  { -1774349.49932, 169481.741461, -442082.866383  , 810 },
                  { -1771921.81299, 210500.067211, -431530.923735  , 1400 },
                  { -1764368.9214, 292232.818166, -409766.263125   , 1206 },
                  { -1759310.48475, 332931.528544, -398768.603927  , 1401 },
                  { -1753240.35156, 373337.733752, -387203.426007  , 1603 },
                  { -1746321.86555, 413655.356459, -375601.941935  , 1800 },
                  { -1738529.81127, 453729.665234, -363846.241034  , 2040 },
                  { -1729842.27977, 493543.77919, -351840.240046   , 2211 },
                  { -1720274.41758, 533113.612079, -339702.758471  , 2420 },
                  { -1709829.36359, 572404.259737, -327390.598492  , 2620 },
                  { -1698540.04418, 611375.1348, -314928.796111    , 2830 },
                  { -1686351.36885, 650102.730545, -302276.447931  , 3040 },
                  { -1673309.3917, 688473.551862, -289489.409978   , 3220 },
                  { -1659408.69974, 726498.59767, -276542.367017   , 3410 },
                  { -1629109.22995, 801388.153814, -250262.985605  , 3600 },
                  { -1595497.67887, 874614.674288, -223481.548309  , 3800 },
                  { -1538978.18196, 981135.55972, -182448.441827   , 4010 },
                  { -1518548.00375, 1015697.88063, -168577.11647   , 4200 },
                  { -1497358.93478, 1049712.83928, -154624.322404  , 4400 },
                  { -1475421.46897, 1083201.24252, -140613.387338  , 4600 },
                  { -1452683.61341, 1116134.66998, -126416.091907  , 4800 },
                  { -1328414.5553, 1272071.66459, -55145.9778978  , 400 } };


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
      //vw::Vector3 temp(k*1000+k, 10, 10);
      vw::Vector3 temp(tmp[k][0],
                        tmp[k][1],
                        tmp[k][2]);

     
      OrbitalReading r_temp("", tmp[k][4] , tmp[k][0]+k, tmp[k][1]+k, tmp[k][2]+k);
      if(k == 4) {
         OrbitalReading r_temp("", tmp[k][4], tmp[k][0]+k+1000, tmp[k][1]+k+100, tmp[k][2]+k+20);
      } 
      
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

TEST( WeightCalculator, OneOutlier )
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

      if (k == 5) {
        OrbitalReading r_temp("", k*20000, tmp[k][0], tmp[k][1], tmp[k][2]);
        readings.push_back(r_temp);
      } else {
        //OrbitalReading r_temp("", k*20000, k*1000, 0, 0);
        OrbitalReading r_temp("", k*20000, tmp[k][0], tmp[k][1], tmp[k][2]);
        readings.push_back(r_temp);
      }

      estimated[k] = temp;
      weights[k] = 0.5;
  }

  // initialize and call weight calculation
  WeightCalculator calc;

  calc.calculateWeights(readings, estimated, weights);

  // make sure all the weights are one
  for(int k = 0; k < 9; k++)
  {
    if (k == 5) {
        EXPECT_EQ(weights[k], 0);
    } else {
        EXPECT_EQ(weights[k], 1);
    }
  }
}

TEST( WeightCalculator, ThreeOutliers )
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

      if (k == 3 || k == 5 || k == 7) {
        OrbitalReading r_temp("", k*20000, k*1500, 0, 0);
        readings.push_back(r_temp);
      } else {
        OrbitalReading r_temp("", k*20000, k*1000, 0, 0);
        readings.push_back(r_temp);
      }

      estimated[k] = temp;
      weights[k] = 0.5;
  }

  // initialize and call weight calculation
  WeightCalculator calc;

  calc.calculateWeights(readings, estimated, weights);

  // make sure all the weights are one
  for(int k = 0; k < 9; k++)
  {
    if (k == 3 || k == 5 || k == 7) {
        EXPECT_EQ(weights[k], 0);
    } else {
        EXPECT_EQ(weights[k], 1);
    }
  }
}
