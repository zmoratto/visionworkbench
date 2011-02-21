#ifndef TRAJECTORY_DECISION_VARIABLE_SET_HPP
#define TRAJECTORY_DECISION_VARIABLE_SET_HPP

#include <vector>
#include <vw/Math/Vector.h>
#include <vw/orbital_refinement/OrbitalReading.hpp>

struct TrajectoryDecisionVariableSet
{
  double GM;
  vw::Vector3 p0;
  vw::Vector3 v0;
  std::vector<OrbitalReading::timestamp_t> timestamps;

  TrajectoryDecisionVariableSet(double GM_in,
                                const vw::Vector3& p0_in,
                                const vw::Vector3& v0_in,
                                const std::list<OrbitalReading>& observations);

  TrajectoryDecisionVariableSet();
};


inline TrajectoryDecisionVariableSet::TrajectoryDecisionVariableSet(
    double GM_in,
    const vw::Vector3& p0_in,
    const vw::Vector3& v0_in,
    const std::list<OrbitalReading>& observations)
        : GM(GM_in), p0(p0_in), v0(v0_in)
{
  timestamps.reserve(observations.size());
  for(std::list<OrbitalReading>::const_iterator it = observations.begin();
      it != observations.end();
      ++it)
    timestamps.push_back(it->mTime);
}

inline TrajectoryDecisionVariableSet::TrajectoryDecisionVariableSet() 
{}
  

#endif
