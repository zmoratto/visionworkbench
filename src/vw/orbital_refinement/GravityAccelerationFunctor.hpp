#ifndef GRAVITY_ACCELERATION_FUNCTOR_HPP
#define GRAVITY_ACCELERATION_FUNCTOR_HPP

#include <vw/Math/Vector.h>

namespace GravityConstants
{
    //! Gravitational Constant for the moon's mass,
    //! in m^3/s^2
    const double GM_MOON = 6.67259e-11*7.36e22;
    //! Gravitational Constant for the moon's mass,
    //! in m^3/millisecond^2
    const double GM_MOON_MILLISECOND = 6.67259e-11*7.36e22/1e6;
}


//! A function object which calculates the acceleration experienced
//! by an entity at some position relative to a gravitational body.
class GravityAccelerationFunctor
{
public:

    //! Constructor.
    //! \param GM The gravitational constant to use.
    //! Defaults to GM_MOON
  GravityAccelerationFunctor(double GM=GravityConstants::GM_MOON)
          : _GM(GM)
      {}
  

    //! Change the gravitational constant.
  void setGM(double GM)
      { _GM = GM; }

    //! Calculate the acceleration felt by an entity at a given point.
    //! \param x The position of the entity relative to the center
    //!        of the gravitational body.  Units should be consistent
    //!        with the functor's current GM value.
  vw::Vector3 operator()(const vw::Vector3& x) const
      {
        double r_squared = dot_prod(x, x);
        double r = sqrt(r_squared);
        return x*(-_GM/(r*r_squared));
      }

private:
  double _GM;
};


#endif
