#ifndef __VW_ORBA_OBSERVATION_SET_HPP
#define __VW_ORBA_OBSERVATION_SET_HPP

#include <vw/ORBA/OrbitalCameraReading.hpp>
#include <vw/BundleAdjustment/ControlNetwork.h>

namespace vw {
namespace ORBA {

using namespace vw;
using namespace vw::ba;
        
// This is a structure in which to store our observations.
// It boils down to a std::vector<OrbitalCameraObservation>
// and a ControlNetwork.
class ObservationSet
{
public:

    // Indicate how many cameras to prepare for.  For efficiency.
  void setExpectedReadingCount(std::size_t reading_count)
  {
    mReadings.reserve(reading_count);
  }
  
  void addReading(const OrbitalCameraReading& pt)
  {
    mReadings.push_back(pt);
  }
        
  
  void setControlNetwork(boost::shared_ptr<ControlNetwork> ctrl_net)
  {
    mControlNetwork = ctrl_net;
  }

  boost::shared_ptr<ControlNetwork> getControlNetwork() const
  {
    return mControlNetwork;
  }

  boost::shared_ptr<PinholeModel> getCamera(std::size_t index) const
  {
    return mReadings[index].mCamera;
  }

  const std::vector<OrbitalCameraReading>& getReadings() const
  {
    return mReadings;
  }

  const OrbitalCameraReading& getReading(std::size_t index) const
  {
    return mReadings[index];
  }
  
private:

  std::vector<OrbitalCameraReading> mReadings;
  
  boost::shared_ptr<ControlNetwork> mControlNetwork;
};
        
} }

#endif
