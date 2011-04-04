#include <ORBARefiner.hpp>
#include <vw/orbital_refinement/DataRefiner.hpp>
#include <vw/orbital_refinement/OrbitalReading.hpp>
#include <OrbitalCameraReading.hpp>

#include <list>

using namespace vw::ba;

namespace vw{
namespace ORBA{

    void normalizeReadingsByTime(std::list<OrbitalCameraReading>& readings)
    {
        // Sort the readings by timestamp.
        readings.sort(OrbitalReading::TimestampLess());

        // Normalize the time readings by subtracting the lowest time
        // from all of the timestamps.
        OrbitalReading::timestamp_t min_time = readings.begin()->mTime;
        for (std::list<OrbitalCameraReading>::iterator it = readings.begin();
          it != readings.end();
          it++)
        {
          it->mTime -= min_time;
        }
    }

    Vector3 estimateInitialVelocity(const std::list<OrbitalCameraReading>& readings)
    {
          // We base the initial velocity on the first 5 readings, or fewer if there
          // are less than 5 available.
        int reading_count = (readings.size() > 5) ? 5 : readings.size();

          // Move to the 5th reading
        std::list<OrbitalCameraReading>::const_iterator it = readings.begin();
        std::advance(it, reading_count-1);
        Vector3 p_diff = it->mCoord - readings.begin()->mCoord;
        OrbitalReading::timestamp_t t_diff = it->mTime - readings.begin()->mTime;

          // Velocity is now just distance / time.
          // Note that you get the exact same result as if you took the
          // average distance / average time:
          //   ((x2-x1 + x1-x0)/2) / ((t2-t1 + t1-t0)/2) == (x2-x0)/(t2-t0)
        p_diff /= t_diff;
        return p_diff;
    }

    bool refineORBAReadings(ControlNetwork& cnet,
            std::list<OrbitalCameraReading>& readings,
            std::list<OrbitalCameraReading>& refined)
    {

        OrbitalRefiner orRefiner = new OrbitalRefiner();

        // Sort by time
        readings.sort(OrbitalReading::TimestampLess);

        // Remember the time we're normalizing on, then normalize
        OrbitalReading::timestamp_t min_time = readings.front().mTime;
        normalizeReadingsByTime(readings);

        // Get an initial velocity
        Vector3 v0 = estimateInitialVelocity(readings);

    }

}} // vw::ORBA