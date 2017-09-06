#ifndef DEPTH_TRACKER_CPP
#define DEPTH_TRACKER_CPP

#include <math.h>

using namespace semantic_mapping::engine;

DepthTracker::DepthTracker(Vector2i image_size, 
                           TrackerIterationType* tracking_regime,
                           int num_hierarchy_levels, 
                           int num_icp_run_till_level,
                           float distance_th,
                           float termination_th,
                           const LowLevelEngine* low_level_engine,
                           MemoryDeviceType memory_type) {
  view_hierarchy_ = new ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >(
                    image_size, tracking_regime, num_hierarchy_levels, 
                    memory_type, true);
  scene_hierarchy_ = new ImageHierarchy<SceneHierarchyLevel>(image_size,
                     tracking_regime, num_hierarchy_levels, memory_type, true);
  this->num_iterations_per_level_ = new int[num_hierarchy_levels];
  this->distance_threshold_ = new float[num_hierarchy_levels];
  this->num_iterations_per_level_[0] = 2;
  for(int level=1; level<num_hierarchy_levels; level++)
    num_iterations_per_level_[level] = num_iterations_per_level_[level-1]+2;
  float dist_th_step = distance_th/num_hierarchy_levels; 
  this->distance_threshold_[num_hierarchy_levels-1] = distance_th;
  for(int level=num_hierarchy_levels-2; level>=0; level--)
    this->distance_threshold_[level] 
        = this->distance_threshold_[level+1]-dist_th_step;
  this->low_level_engine_ = low_level_engine;
  this->num_icp_level_ = num_icp_run_till_level;
  this->termination_threshold_ = termination_th;
}

~DepthTracker::~DepthTracker() {
  delete this->view_hierarchy_;
  delete this->scene_hierarchy_;
  delete[] this->num_iterations_per_level_;
  delete[] this->distance_threshold_;
}

void DepthTracker::SetEvaluationData(TrackingState* tracking_state, 
                                     const View* view) {
  this->tracking_state_ = tracking_state;
  this->view_ = view;
  scene_hierarchy_->levels_[0]->intrinsics_ 
       = view_->rgbd_calib_->intrinsics_depth_.ProjectionParameters.all;
  view_hierarchy_->levels_[0]->intrinsics_
       = view_->rgbd_calib_->intrinsics_depth_.ProjectionParameters.all;
  view_hierarchy_->levels_[0]->depth_ = view_->depth_;
  scene_hierarchy_->levels_[0]->points_map_ 
                  = tracking_state_->point_cloud_->locations_;
  scene_hierarchy_->levels_[0]->normals_map_ 
                  = tracking_state_->point_cloud_->colors_;
  scene_pose_ = tracking_state_->pose_point_cloud_->GetMatrix();
}



#endif /* end of include guard: DEPTH_TRACKER_CPP */
