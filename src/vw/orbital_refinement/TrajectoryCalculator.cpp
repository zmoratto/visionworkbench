
#include <vw/Math/RungeKutta.h>
#include <vw/orbital_refinement/TrajectoryCalculator.hpp>
#include <vw/orbital_refinement/GravityAccelerationFunctor.hpp>

namespace
{
    struct PositionVelocity
    {
      vw::Vector3 position;
      vw::Vector3 velocity;
    };

    PositionVelocity operator+(const PositionVelocity& lhs,
                               const PositionVelocity& rhs)
    {
      PositionVelocity rv;
      rv.position = lhs.position + rhs.position;
      rv.velocity = lhs.velocity + rhs.velocity;
      return rv;
    }
    
    PositionVelocity operator/(const PositionVelocity& lhs,
                               int rhs)
    {
      PositionVelocity rv;
      rv.position = lhs.position / rhs;
      rv.velocity = lhs.velocity / rhs;
      return rv;
    }
    
    struct VelocityAcceleration
    {
      vw::Vector3 velocity;
      vw::Vector3 acceleration;
    };

      // Return delta position and delta velocity if the given
      // acceleration is applied to the starting velocity for
      // the given amount of time.
    PositionVelocity scale(const VelocityAcceleration& va,
                           OrbitalReading::timestamp_t time)
    {
      PositionVelocity pv;
        // Remember, time is in milliseconds.
      pv.velocity = va.acceleration * time / 1000;
      pv.position = (va.velocity + .5*pv.velocity)*time / 1000;
      return pv;
    }
    

    
    struct GravityAdapter
    {
      GravityAdapter(const GravityAccelerationFunctor& accelerator)
              : _accelerator(accelerator)
          {}

        //! Given (time, {position_velocity}),
        //! return velocity and acceleration at that point.
        //! Velocity won't change (it is at a point); acceleration
        //! is determined by the accelerator.
      VelocityAcceleration operator ()(
          OrbitalReading::timestamp_t time,
          const PositionVelocity& position_velocity)
      {
        VelocityAcceleration va;
        va.velocity = position_velocity.velocity;
        va.acceleration = _accelerator(position_velocity.position);
        return va;
      }

      const GravityAccelerationFunctor& _accelerator;
    };
}

      

void TrajectoryCalculator::calculateNextPoint(
    vw::Vector3 cur_location,
    vw::Vector3 cur_velocity,
    OrbitalReading::timestamp_t time_delta,
    vw::Vector3& next_location,
    vw::Vector3& next_velocity)
{
  vw::math::RungeKutta rk;

  GravityAccelerationFunctor grav_func(GravityConstants::GM_MOON);
  GravityAdapter grav_adapter(grav_func);
  PositionVelocity start_state;
  start_state.position = cur_location;
  start_state.velocity = cur_velocity;
  
  PositionVelocity end_state =
      rk.solve(OrbitalReading::timestamp_t(0),
               start_state,
               time_delta,
               grav_adapter,
               &scale);
  next_location = end_state.position;
  next_velocity = end_state.velocity;
}
