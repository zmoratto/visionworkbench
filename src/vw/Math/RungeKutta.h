// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __VW_MATH_RUNGE_KUTTA_H__
#define __VW_MATH_RUNGE_KUTTA_H__

#include <vector>

namespace vw {
namespace math{

class RungeKutta
{
public:

  //! Given f(x0)=y0, find f(x0+dx).
  //! \param x0 Initial domain value.
  //! \param y0 Initial function value, f(x0)
  //! \param dx The desired step size
  //! \param derivatives Functor which returns the derivative of f(x,y).  
  //!        Must define operator()(DomainT x, ResultT y).
  //! \returns An estimated value of f(x0+dx)
  template <typename DomainT, typename ResultT, typename DerivativeFunctorT>
  ResultT solve(const DomainT& x0, const ResultT& y0, 
                const DomainT& dx,
                DerivativeFunctorT derivative);
};

template <typename DomainT, typename ResultT, typename DerivativeFunctorT>
ResultT RungeKutta::solve(const DomainT& x0, const ResultT& y0, 
                          const DomainT& dx,
                          DerivativeFunctorT derivative)
{
  // A)  k1 = deriv(x, y) * dx
  ResultT k1 = derivative(x0, y0) * dx;

  // B) y2 = (y + k1/2)
  ResultT y2 = y0 + k1/2;

  // C) k2 = deriv(x + dx/2, y2) * dx
  ResultT k2 = derivative(x0+dx/2, y2) * dx;

  // D) y3 = y0 + k2/2
  ResultT y3 = y0 + k2/2;

  // E) k3 = deriv(x+dx/2, y3) * dx
  ResultT k3 = derivative(x0+dx/2, y3) * dx;

  // F) y4 = y + k3
  ResultT y4 = y0 + k3;

  // G) k4 = deriv(x+dx, y4) * dx
  ResultT k4 = derivative(x0+dx, y4) * dx;

  // Add them up to get the final answer
  ResultT y_final = y0 + (k1+k4)/6 + (k2+k3)/3;

  return y_final;
}

} } // namespace vw::math

#endif
