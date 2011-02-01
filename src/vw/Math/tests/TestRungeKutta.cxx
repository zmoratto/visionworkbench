// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// TestConjugateGradient.h
#include <gtest/gtest.h>
#include <vw/Math/RungeKutta.h>
#include <vw/Math/Vector.h>
#include <cmath>
#include <boost/bind.hpp>

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
  double dx = .2;

  RungeKutta rk;
  for (double x = 0; x < 10; x += dx)
  {
    double new_val = rk.solve(x, sin(x) + 2.2, dx, my_derivative, std::multiplies<double>());
    EXPECT_NEAR(new_val, sin(x+dx) + 2.2, 1e-3);
  }
}


struct QuadraticFunction
{
  typedef double result_type;
  typedef Vector2 domain_type;
  typedef Vector2 gradient_type;

  result_type operator()( domain_type const& x ) const
  {
    return 1.2 * pow(x[0] - 0.6, 2) 
      + 1.7 * pow(x[1] - 0.6, 2) 
      + 2 * x[0] * x[1];
  }

  static gradient_type gradient( domain_type const& x )
  {
    return Vector2( 2.4*x[0]-1.44+2*x[1],
                    3.4*x[1]-2.04+2*x[0]);
  }
};

QuadraticFunction::gradient_type gradient_transform(
  const QuadraticFunction::domain_type& x, 
  const QuadraticFunction::result_type& y)
{
  return QuadraticFunction::gradient(x);
}

QuadraticFunction::result_type scale_gradient(
  const QuadraticFunction::domain_type& gradient,
  const QuadraticFunction::domain_type& dx)
{
  return dot_prod(gradient, dx);
}


// Test that has a 2D domain and a scalar result.
TEST(RungeKutta, RungeKutta2DDomain)
{
  // use Runge-Kutta to evaluate f(x,y) = QuadraticFunction defined above.
  QuadraticFunction::domain_type x(.2, .5);
  QuadraticFunction::domain_type dx(.2, .5);
  QuadraticFunction::result_type tolerance =  1e-3;
  QuadraticFunction f;

  RungeKutta rk;
  // try 10 values
  for (int i = 0; i < 10; ++i)
  {
    QuadraticFunction::result_type new_val = 
        rk.solve(x, f(x), dx, gradient_transform, scale_gradient);
    EXPECT_NEAR(new_val, f(x+dx), tolerance);
    x += dx;
  }
}


// Function that has a 2D domain and a 2D result
struct QuadraticFnQuadraticResult
{
  Vector2 operator()(const Vector2& x)
      {
        return Vector2(sin(x[0]) - x[0]*x[0]*x[1] + 2*x[1]*x[1],
                       4*x[0]*x[1] + 3*x[0]*x[1]*x[1]);
      }

  static Vector<Vector2,2> gradient(const Vector2& x, const Vector2& y)
      {
        Vector<Vector2,2> result(Vector2(0,0), Vector2(0,0));
        result[0][0] = cos(x[0]) - 2*x[0]*x[1];
        result[0][1] = -x[0]*x[0] + 4*x[1];
        result[1][0] = 4*x[1] + 3*x[1]*x[1];
        result[1][1] = 4*x[0] + 6*x[0]*x[1];
        return result;
      }
  
  static Vector2 scale_gradient(const Vector<Vector2,2>& g, const Vector2& dx)
      {
        return Vector2(dot_prod(g[0], dx), dot_prod(g[1], dx));
      }
};

// Test that has a 2D domain and a 2D result
TEST(RungeKutta, RungeKutta2DResult)
{
  // use Runge-Kutta to evaluate f(x,y) = QuadraticFunction defined above.
  Vector2 x(.2, .5);
  Vector2 dx(.2, .5);
  double tolerance = 1e-3;
  QuadraticFnQuadraticResult f;

  RungeKutta rk;
  // try 10 values
  for (int i = 0; i < 10; ++i)
  {
    Vector2 new_val = 
        rk.solve(x, f(x), dx, QuadraticFnQuadraticResult::gradient, QuadraticFnQuadraticResult::scale_gradient);
    EXPECT_NEAR(new_val[0], f(x+dx)[0], tolerance);
    EXPECT_NEAR(new_val[1], f(x+dx)[1], tolerance);
    x += dx;
  }
}
