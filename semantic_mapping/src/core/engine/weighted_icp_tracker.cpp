#include "weighted_icp_tracker.h"

#include <math.h>

using namespace semantic_mapping::engine;

WeightedIcpTracker::WeightedIcpTracker(Vector2i image_size, 
                                       TrackerIterationType* tracking_regime,
                                       int num_hierarchy_levels,
                                       int num_icp_run_till_level,
                                       float distance_th,
                                       float termination_th,
                                       const LowLevelEngine* low_level_engine,
                                       MemoryDeviceType memory_type) {
  if(memory_type == MEMORY_DEVICE_GPU) {
    weight_hierarchy_ = new ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >
      (image_size, tracking_regime, num_hierarchy_levels, memory_type, true);
    view_hierarchy_ = new ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >
      (image_size, tracking_regime, num_hierarchy_levels, memory_type, true);
    scene_hierarchy_ = new ImageHierarchy<SceneHierarchyLevel>(image_size,
      tracking_regime, num_hierarchy_levels, memory_type, true);
  } else {
    weight_hierarchy_ = new ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >
      (image_size, tracking_regime, num_hierarchy_levels, memory_type, true);
    view_hierarchy_ = new ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >
      (image_size, tracking_regime, num_hierarchy_levels, memory_type, true);
    scene_hierarchy_ = new ImageHierarchy<SceneHierarchyLevel>(image_size,
      tracking_regime, num_hierarchy_levels, memory_type, true);
  }
  this->num_iterations_per_level_ = new int[num_hierarchy_levels];
  this->distance_threshold_ = new float[num_hierarchy_levels];
  this->num_iterations_per_level_[0] = 2;

  for(int level=1; level<num_hierarchy_levels; level++)
    num_iterations_per_level_[level] = num_iterations_per_level_[level-1]+2;
  
  float dist_th_step = distance_th/num_hierarchy_levels;
  this->distance_threshold_[num_hierarchy_levels-1] = distance_th;
  for(int level=num_hierarchy_levels-2; level>=0; level--) 
    this->distance_threshold_[level] = this->distance_threshold_[level+1]
                                       -dist_th_step; 
  this->low_level_engine_ = low_level_engine;
  this->num_icp_level_ = num_icp_run_till_level;
  this->termination_threshold_ = termination_th;
}

WeightedIcpTracker::~WeightedIcpTracker() {
  delete this->view_hierarchy_;
  delete this->weight_hierarchy_;
  delete this->scene_hierarchy_;
  delete[] this->num_iterations_per_level_; 
}

void WeightedIcpTracker::SetEvaluationData() {
  for(int i=0; i<view_hierarchy_->num_levels_; i++) {
    TemplatedHierarchyLevel<FloatImage>* current_wicp_level 
                                         = view_hierarchy_->levels_[i];
    TemplatedHierarchyLevel<FloatImage>* previous_wicp_level
                                         = view_hierarchy_->levels_[i-1];
    low_level_engine_->FilterSubsampleWithHoles(current_wicp_level->depth_,
                                                previous_wicp_level->depth_);
    current_wicp_level->intrinsics_= previous_wicp_level->intrinsics_*0.5f;
    current_wicp_level = weight_hierarchy_->levels_[i];
    previous_wicp_level = weight_hierarchy_->levels_[i-1];
    low_level_engine_->FilterSubsampleWithHoles(current_wicp_level->depth_,
                                                previous_wicp_level->depth_);
    SceneHierarchyLevel* current_level_scene = scene_hierarchy_->levels_[i];
    SceneHierarchyLevel* previous_level_scene = scene_hierarchy_->levels_[i-1];
    current_level_scene->intrinsics_ = previous_level_scene->intrinsics_*0.5f;
  }
}

void WeightedIcpTracker::SetEvaluationParameters(int level) {
  this->level_id_ = level;
  this->iteration_type_ = view_hierarchy_->levels_[level]->iteration_type_;
  this->scene_hierarchy_level_ = scene_hierarchy_->levels_[0];
  this->view_hierarchy_level_ = view_hierarchy_->levels_[level];
  this->weight_hierarchy_level_ = weight_hierarchy_->levels_[level];
}

void WeightedIcpTracker::ComputeDelta(float* step, float* nabla, 
                         float* hessian, bool short_iteration) const {
  for(int i=0; i<6; i++)
    step[i] = 0;
  if(short_iteration) {
    float small_hessian[9];
    for(int r=0; r<3; r++)
      for(int c=0; c<3; c++)
        small_hessian[r+c*3] = hessian[r+c*6];
    utils::Cholesky cholsky(small_hessian, 3);
    cholsky.BackSub(step, nabla);
  } else {
    utils::Cholesky cholesky(hessian, 6);
    cholsky(step, nabla);
  }
}

bool WeightedIcpTracker::HasConverged(float* step) const {
  float step_length = 0.0f;
  for(int i=0; i<6; i++)
    step_length += step[i]*step[i];
  if(sqrt(step_length)/6 < termination_threshold_)
    return true;
  return false;
}

void WeightedIcpTracker::ApplyDelta(const Matrix4f& old_pose, 
                                    const float* delta, 
                                    Matrix4f& new_pose) const {
  float step[6];                                                              
  switch (iterationType) {                                                                           
    case TRACKER_ITERATION_ROTATION:                                            
      step[0] = (float)(delta[0]); 
      step[1] = (float)(delta[1]); 
      step[2] = (float)(delta[2]);
      step[3] = 0.0f; 
      step[4] = 0.0f; 
      step[5] = 0.0f;                         
      break;                                                                  
    case TRACKER_ITERATION_TRANSLATION:                                         
      step[0] = 0.0f; 
      step[1] = 0.0f; 
      step[2] = 0.0f;                         
      step[3] = (float)(delta[0]); 
      step[4] = (float)(delta[1]); 
      step[5] = (float)(delta[2]);
      break;                                                                  
    default:                                                                    
    case TRACKER_ITERATION_BOTH:                                                
      step[0] = (float)(delta[0]); 
      step[1] = (float)(delta[1]); 
      step[2] = (float)(delta[2]);
      step[3] = (float)(delta[3]); 
      step[4] = (float)(delta[4]); 
      step[5] = (float)(delta[5]);
      break;                                                                  
  }                                                                           
  Matrix4f Tinc;                                                              
  Tinc.m00 = 1.0f;        
  Tinc.m10 = step[2];     
  Tinc.m20 = -step[1];    
  Tinc.m30 = step[3];
  Tinc.m01 = -step[2];    
  Tinc.m11 = 1.0f;        
  Tinc.m21 = step[0];     
  Tinc.m31 = step[4];
  Tinc.m02 = step[1];     
  Tinc.m12 = -step[0];    
  Tinc.m22 = 1.0f;        
  Tinc.m32 = step[5];
  Tinc.m03 = 0.0f;        
  Tinc.m13 = 0.0f;        
  Tinc.m23 = 0.0f;        
  Tinc.m33 = 1.0f;
  
  new_pose = Tinc * old_pose;                                                 
}

void WeightedIcpTracker::TrackCamera(TrackingState* tracking_state, 
                                     const View* view) {
  this->SetEvaluationData(tracking_state, view);
  this->PrepareForEvaluation();
  Matrix4f approx_inv_pose = tracking_state->pose_depth_->GetInverseMatrix();
  float f_old = 1e10;
  float f_new;
  
  for(int level=view_hierarchy_->num_levels_-1; level>=num_icp_level_; level--) {
    this->SetEvaluationParameters(level);
    if(iteration_type_ == TRACKER_ITERATION_NONE)
      continue;
    for(int iter=0; iter<num_iterations_per_level_[level]; iter++) {
      int num_valid_points = this->ComputeGrandH(f_new, nabla_, hessian_, 
                                                 approx_inv_pose);
      if(num_valid_points<=0)
        break;
      if(f_new > f_old)
        break;
      ComputeDelta(step_, nabla_, hessian_, 
                   iteration_type_ != TRACKER_ITERATION_BOTH);
      ApplyDelta(approx_inv_pose, step_, approx_inv_pose);
      tracking_state_->pose_depth_->SetInverseMatrix(approx_inv_pose);
      tracking_state_->pose_depth_->Coerce();
      approx_inv_pose = tracking_state_->pose_depth_->GetInverseMatrix();
      if(HasConverged(step))
        break; 
    }
  }
}
