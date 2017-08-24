#ifndef SETTINGS_H
#define SETTINGS_H

namespace semantic_mapping {
namespace objects {

class Settings {
public:
  typedef enum { DEVICE_CPU, DEVICE_GPU} DeviceType;
  DeviceType device_type_;
  bool use_swapping_;
  bool use_approximate_raycast_;
  bool use_bilateral_filter_;
  bool model_sensor_noise_;
  
  typedef enum { 
    TRACKER_COLOR, TRACKER_ICP, TRACKER_REN, TRACKER_IMU, TRACKER_WICP
  } TrackerType;
  TrackerType tracker_type_;
  TrackerIterationType* tracking_regime_;
  int num_hierarchy_levels_;
  int num_icp_till_level_;
  bool skip_points_;
  float depth_tracker_icp_threshold_;
  float depth_tracker_termination_threshold_;
  semantic_mapping::objects::SceneParameters scene_parameters_;
  Settings();
  ~Settings();
  Settings(const Settings&);
  Settings& operator=(const Settings&);  
};
		
} /* objects */ 
} /* semantic_mapping */ 

#endif /* end of include guard: SETTINGS_H */
