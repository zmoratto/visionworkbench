#ifndef __VW_ORBA_OBSERVATION_SET_HPP
#define __VW_ORBA_OBSERVATION_SET_HPP

#include <vw/ORBA/OrbitalCameraReading.hpp>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vector>
#include <map>

namespace vw {
namespace ORBA {

using namespace vw;
using namespace vw::ba;
        
// This is a structure in which to store our observations.
// It boils down to a std::vector<OrbitalCameraReading>
// and a ControlNetwork.
// Readings and associated cameras are accessible by index,
// which is the 0-n order in the vector, or by Camera ID,
// which is the image_id() it is referred to in the ControlNetwork.
// You can get the index from the image ID using getIndexFromImageID().
// The OrbitalCameraReading class 
class ObservationSet
{
public:

    // Default constructor
  ObservationSet()
          : mBaseTime(0)
  {}

    // Indicate how many cameras to prepare for.  For efficiency.
  void setExpectedReadingCount(std::size_t reading_count)
  {
    mReadings.reserve(reading_count);
  }
  
  void addReading(const OrbitalCameraReading& pt)
  {
    mIndexMap[pt.mImageID] = mReadings.size();
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

  boost::shared_ptr<PinholeModel> getCameraByImageID(std::size_t image_id) const
  {
    if (mIndexMap.find(image_id) == mIndexMap.end())
      std::cout << " Tried to get a camera with ID " << image_id << ", which we don't have" << std::endl;
    return mReadings[mIndexMap.find(image_id)->second].mCamera;
  }

  boost::shared_ptr<PinholeModel> getCameraByIndex(std::size_t index) const
  {
    return mReadings[index].mCamera;
  }

  const std::vector<OrbitalCameraReading>& getReadings() const
  {
    return mReadings;
  }

  const OrbitalCameraReading& getReadingByImageID(std::size_t index) const
  {
    return mReadings[mIndexMap.find(index)->second];
  }

  const OrbitalCameraReading& getReadingByIndex(std::size_t index) const
  {
    return mReadings[index];
  }

  std::size_t getIndexFromImageID(std::size_t image_id) const
  { return mIndexMap.find(image_id)->second; }

  bool imageIDExists(std::size_t image_id) const
  { return mIndexMap.find(image_id) != mIndexMap.end(); }

  void normalizeTimes()
  {
    mBaseTime = mReadings.begin()->mTime;
    if (mBaseTime == 0)
      return;
    
    for (std::vector<OrbitalCameraReading>::iterator it = mReadings.begin();
         it != mReadings.end();
         it++)
    {
      it->mTime -= mBaseTime;
    }
  }

  void denormalizeReadingTimes()
  {
    if (mBaseTime == 0)
      return;
    
    for (std::vector<OrbitalCameraReading>::iterator it = mReadings.begin();
         it != mReadings.end();
         it++)
    {
      it->mTime += mBaseTime;
    }
    mBaseTime = 0;
  }
  
  OrbitalReading::timestamp_t getTimeNormalizationBase() const
  { return mBaseTime; }

  void setTimeNormalizationBase(OrbitalReading::timestamp_t base)
  { mBaseTime = base; }

private:

  std::map<std::size_t, std::size_t> mIndexMap;

  std::vector<OrbitalCameraReading> mReadings;
  
  boost::shared_ptr<ControlNetwork> mControlNetwork;

    // When times are normalized, this is the amount
    // that has been subtracted from each time.
  OrbitalReading::timestamp_t mBaseTime;
};

} }

#endif
