#ifndef IMU_TRACKER_H
#define IMU_TRACKER_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
	
class ImuTracker {
public:
  void TrackCamera(TrackingState* tracking_state, const View* view);
  
  ImuTracker(ImuCalibrator* imu_calib);
  
  virtual ~ImuTracker();

private:
  ImuCalibrator* imu_calib_;
};
	
} /* engine */ 
} /* semantic_mapping */ 



#endif /* end of include guard: IMU_TRACKER_H */
