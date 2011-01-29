// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// TestConjugateGradient.h
#include <gtest/gtest.h>
#include <vw/Math/RungeKutta.h>
#include <cmath>

using namespace vw;
using namespace vw::math;

// Function to calculate derivative of sin(x), which is cos(x)
double my_derivative(double x, double y)
{
  return cos(x);
}

TEST( RungeKutta, SimpleRungeKutta )
{
  // use Runge-Kutta to evaluate f(x) = sin(x) + 2.2
  double x = 0;
  double y = 2.2;
  double dx = .2;

  RungeKutta rk;
  for (double x = 0; x < 10; x += dx)
  {
    double new_val = rk.solve(x, y, dx, my_derivative);
    EXPECT_NEAR(new_val, sin(x) + 2.2, 1e-3);
  }
}

