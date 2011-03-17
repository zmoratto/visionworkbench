#ifndef TRAJECTORY_GRADIENT_SET_HPP
#define TRAJECTORY_GRADIENT_SET_HPP

#include <vector>
#include <vw/Math/Vector.h>
#include <vw/orbital_refinement/TrajectoryDecisionVariableSet.hpp>
#include <iostream>

struct TrajectoryGradientSet
{
  double GM;
  vw::Vector3 p0;
  vw::Vector3 v0;
  std::vector<double> t;

  TrajectoryGradientSet()
  {}

  TrajectoryGradientSet& operator=(const TrajectoryGradientSet& rhs)
  {
    GM = rhs.GM;
    p0 = rhs.p0;
    v0 = rhs.v0;
      // This next line makes a copy.
      // Consider doing a swap if we want to speed this up,
      // but it will invalidate rhs.
    t = rhs.t;

    return *this;
  }

  TrajectoryGradientSet operator-()
  {
    TrajectoryGradientSet result;
    result.GM = -GM;
    result.p0 = -p0;
    result.v0 = -v0;
    result.t.resize(t.size());
    for (std::size_t i = 0; i < t.size(); ++i)
      result.t[i] = -(t[i]);
    return result;
  }
};

inline double dot_prod(const TrajectoryGradientSet& lhs, const TrajectoryGradientSet& rhs)
{
  double result = lhs.GM*rhs.GM;
  result += dot_prod(lhs.p0, rhs.p0);
  result += dot_prod(lhs.v0, rhs.v0);
  for (std::size_t i = 0; i < lhs.t.size(); ++i)
  {
    result += lhs.t[i]*rhs.t[i];
  }
  
  return result;
}

inline TrajectoryGradientSet operator*(double lhs, const TrajectoryGradientSet& rhs)
{
  TrajectoryGradientSet result;
  result.GM = lhs*rhs.GM;
  result.p0 = lhs*rhs.p0;
  result.v0 = lhs*rhs.v0;
  result.t.resize(rhs.t.size());
  for (std::size_t i = 0; i < rhs.t.size(); ++i)
    result.t[i] = lhs*rhs.t[i];
  return result;
}

inline TrajectoryGradientSet operator+(const TrajectoryGradientSet& lhs,
                                const TrajectoryGradientSet& rhs)
{
  TrajectoryGradientSet result;
  result.GM = lhs.GM + rhs.GM;
  result.p0 = lhs.p0 + rhs.p0;
  result.v0 = lhs.v0 + rhs.v0;
  result.t.resize(rhs.t.size());
  for (std::size_t i = 0; i < rhs.t.size(); ++i)
    result.t[i] = lhs.t[i]+rhs.t[i];
  return result;
}

inline TrajectoryDecisionVariableSet operator+(const TrajectoryDecisionVariableSet& lhs,
                                        const TrajectoryGradientSet& rhs)
{
  TrajectoryDecisionVariableSet result;
  result.GM = lhs.GM + rhs.GM*1e-18;
  result.p0 = lhs.p0 + rhs.p0;
  result.v0 = lhs.v0 + rhs.v0;
  result.timestamps.resize(rhs.t.size());
  for (std::size_t i = 0; i < rhs.t.size(); ++i)
    result.timestamps[i] = lhs.timestamps[i]+rhs.t[i];
  return result;
}

inline TrajectoryGradientSet& operator+=(TrajectoryGradientSet& lhs,
                                  const TrajectoryGradientSet& rhs)
{
  lhs.GM += rhs.GM;
  lhs.p0 += rhs.p0;
  lhs.v0 += rhs.v0;
  for (std::size_t i = 0; i < rhs.t.size(); ++i)
    lhs.t[i] += rhs.t[i];
  return lhs;
}

#endif
