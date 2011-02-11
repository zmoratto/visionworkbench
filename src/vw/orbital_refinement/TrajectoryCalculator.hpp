#ifndef TRAJECTORY_CALCULATOR_HPP
#define TRAJECTORY_CALCULATOR_HPP

#include <vw/Math/Vector.h>
#include <vw/orbital_refinement/OrbitalReading.hpp>

class GravityAccelerationFunctor;

class TrajectoryCalculator
{
public:
  TrajectoryCalculator(GravityAccelerationFunctor& acceleration)
          : _accelerator(&acceleration)
      {}
  
  void setAccelerationFunctor(GravityAccelerationFunctor& acceleration)
      {
        _accelerator = &acceleration;
      }

  void calculateNextPoint(vw::Vector3 cur_location,
                          vw::Vector3 cur_velocity,
                          OrbitalReading::timestamp_t time_delta,
                          vw::Vector3& next_location,
                          vw::Vector3& next_velocity);

private:
  GravityAccelerationFunctor* _accelerator;
};


#endif
