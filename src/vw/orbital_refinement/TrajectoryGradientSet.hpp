#ifndef TRAJECTORY_GRADIENT_SET_HPP
#define TRAJECTORY_GRADIENT_SET_HPP

#include <vector>
#include <vw/Math/Vector.h>

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
};

#endif
