#include <ORBARefiner.hpp>
#include <vw/orbital_refinement/DataRefiner.hpp>
#include <vw/orbital_refinement/OrbitalReading.hpp>
#include <OrbitalCameraReading.hpp>

#include <list>

using namespace vw::ba;

namespace vw{
namespace ORBA{

    bool refineORBAReadings(ControlNetwork& cnet,
            std::list<OrbitalCameraReading>& readings,
            std::list<OrbitalCameraReading>& refined) {

        OrbitalRefiner orRefiner = new OrbitalRefiner();

        // Sort by time
        readings.sort(OrbitalReading::TimestampLess);

        // Remember the time we're normalizing on
        OrbitalReading::timestamp_t min_time = readings.front().mTime;
        
    }

}} // vw::ORBA