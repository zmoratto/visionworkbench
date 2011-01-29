#include "DataRefiner.hpp"
#include "OrbitalReading.hpp"
#include <Math.h>
#include <list>

/*
 *
 */
void refineData(std::list<OrbitalReading> readings) {
    
    // Constants
    double GRAVITY = 6.67259e-11;
    double MASS = 7.36e22;
    double SCALE_GM = sqrt(MASS/GRAVITY);
    double SCALE_G = GRAVITY*SCALE_GM;
    double SCALE_M = MASS*SCALE_GM;
    double GM = GRAVITY*MASS;

    // Iterate through the list of readings
    for (std::list<OrbitalReading>::iterator it = readings.begin();
         it != readings.end(); ++it) {
        
    }
}