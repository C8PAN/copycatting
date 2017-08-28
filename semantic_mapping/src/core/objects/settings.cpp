#include "settings.h"
#include <stdio.h>

using namespace semantic_mapping::objects;

Settings::Settings() : SceneParameters(0.02f, 100, 0.005f, 0.2f, 3.0f, false) {
  depth_tracker_icp_threshold_ = 0.1f*0.1f;
  depth_tracker_termination_threshold_ = 1e-3f;
  skip_points_ = true;
  device_type_ = DEVICE_GPU;
  use_swapping_ = false;
  use_approximate_raycast_ = false;
  use_bilateral_filter_ = false;
  tracker_type_ = TRACKER_ICP;
  model_sensor_noise_ = false;
  
  if(tracker_type_==TRACKER_WICP)
    model_sensor_noise_ = true;
  
  if(tracker_type_==TRACKER_IMU) {
    num_hierarchy_levels_ = 2;
    tracking_regime_ = new TrackerIterationType[num_hierarchy_levels_];
    tracking_regime_[0] = TRACKER_ITERATION_BOTH;
    tracking_regime_[1] = TRACKER_ITERATION_TRANSLATION;
  } 
  else {
    num_hierarchy_levels_ = 5;
    tracking_regime_ = new TrackerIterationType[num_hierarchy_levels_];
    tracking_regime_[0] = TRACKER_ITERATION_BOTH;
    tracking_regime_[1] = TRACKER_ITERATION_BOTH;
    tracking_regime_[2] = TRACKER_ITERATION_ROTATION;
    tracking_regime_[3] = TRACKER_ITERATION_ROTATION;
    tracking_regime_[4] = TRACKER_ITERATION_ROTATION;
  }

  if(tracker_type_==TRACKER_REN)
    num_icp_till_level_ = 1;
  else
    num_icp_till_level_ = 0;

  if((tracker_type_==TRACKER_COLOR) && (!Voxel::has_color_information_)) {
    printf("Error: color tracker requires color information\n");
  }
}

Settings::~Settings() {
  delete [] tracking_regime_;
}
