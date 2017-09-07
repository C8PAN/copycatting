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

void DepthTracker::PrepareForEvaluation() {
  for(int i=1; i<view_hierarchy_->num_levels_; i++) {
    TemplatedHierarchyLevel<FloatImage>* current_level_view 
      = view_hierarchy_->levels_[i];
    TemplatedHierarchyLevel<FloatImage>* previous_level_view
      = view_hierarchy_->levels_[i-1];
    low_level_engine_->FilterSubsampleWithHoles(current_level_view->depth_,
                                                previous_level_view->depth_);
    current_level_view->intrinsics_ = previous_level_view->intrinsics_*0.5f;
    SceneHierarchyLevel* current_level_scene = scene_hierarchy_->levels_[i];
    SceneHierarchyLevel* previous_level_scene = scene_hierarchy_->levels_[i-1];
    current_level_scene->intrinsics_ = previous_level_scene->intrinsics_*0.5f; 
  }
}

void DepthTracker::SetEvaluationParameters(int level) {
  this->level_id = level;
  this->iteration_type = view_hierarchy->levels_[level]->iteration_type_;
  this->scene_hierchy_level_ = scene_hierarchy_->levels_[level];
}

void DepthTracker::ComputeDelta(float* step, float* nabla, float* hessian, 
                                bool short_iteration) const {
  for(int i=0; i<6; i++) 
    step[i] = 0;
  if(short_iteration) {
    float small_hessian[9];
    for(int r=0; r<3; r++)
      for(int c=0; c<3; c++)
        small_hessian[r+c*3] = hessian[r+c*6];
    utils::Cholesky cholesky(small_hessian, 3);
    cholesky.BackSub(step, nabla);
  } else {
    utils::Cholesky cholesky(hessian, 6);
    cholesky.BackSub(step, nabla);
  }
}

DepthTracker::HasConverged(float* step) const {
  float step_length = 0.0f;
  for(itn i=0; i<6; i++)
    step_length += step[i]*step[i];
  if(sqrt(step_length)/6 < termination_threshold_) 
    return true;
  return false;
}

void DepthTracker::ApplyDelta(const Matrix4f& old_parameter, 
                   const float* delta, Matrix4f& new_parameter) const {
  float step[6];
  
  switch(iteration_type_) {
    case TRACKER_ITERATION_ROTATION:
      step[0]=(float)(delta[0]);
      step[1]=(float)(delta[1]);
      step[2]=(float)(delta[2]);
      step[3]=0.0f;
      step[4]=0.0f;
      step[5]=0.0f;
      break;
    case TRACKER_ITERATION_TRANSLATION:
      step[0]=0.0f;
      step[1]=0.0f;
      step[2]=0.0f;
      step[3]=(float)(delta[0]);
      step[4]=(float)(delta[1]);
      step[5]=(float)(delta[2]);
      break;
    default:
    case TRACKER_ITERATION_BOTH:
      step[0]=(float)(delta[0]);
      step[1]=(float)(delta[1]);
      step[2]=(float)(delta[2]);
      step[3]=(float)(delta[3]);
      step[4]=(float)(delta[4]);
      step[5]=(float)(delta[5]);
      break;
  }

  Matrix4f mat_increment;
  mat_increment.m00 = 1.0f;
  mat_increment.m10 = step[2];
  mat_increment.m20 = -step[1];
  mat_increment.m30 = step[3];
  mat_increment.m01 = -step[2];
  mat_increment.m11 = 1.0f;
  mat_increment.m21 = step[0];
  mat_increment.m31 = step[4];
  mat_increment.m02 = step[1];
  mat_increment.m12 = -step[0];
  mat_increment.m22 = 1.0f;
  mat_increment.m32 = step[5];
  mat_increment.m03 = 0.0f;
  mat_increment.m13 = 0.0f;
  mat_increment.m23 = 0.0f;
  mat_increment.m33 = 1.0f;

  new_parameter = mat_increment*old_parameter;
}

void DepthTracker::TrackCamera(TrackingState* tracking_state, 
                               const View* view) {
  this->SetEvaluationData(tracking_state, view);
  this->PrepareForEvaluation();
  float f_old = 1e10;
  float f_new;
  int num_valid_points_new;
  float hessian_good[36];
  float hessian_new[36];
  float A[36];
  float nabla_good[6];
  float nabla_new[6];
  float step[6];

  for(int level=view_hierarchy_->num_levels_-1;level>=num_icp_level_;level--) {
    this->SetEvaluationParameters(level);
    if(iteration_type_ == TRACKER_ITERATION_NONE)
      continue;
    Matrix4f approx_inv_pose = tracking_state->pose_depth_->GetInverseMatrix();
    Pose last_known_good_pose(*(tracking_state->pose_depth_));
    f_old = 1e20f;
    float lambda = 1.0f;
    for(int iter=0; iter<num_iterations_per_level_[level]; iter++) {
      num_valid_points_new = this->ComputeGrandH(f_new, nabla_new, hessian_new,
                                                 approx_inv_pose);
      if((num_valid_points_new<=0) || (f_new > f_old)) {
        tracking_state->pose_depth_->SetFrom(&last_known_good_pose);
        approx_inv_pose = tracking_state->pose_depth_->GetInverseMatrix();
        lambda *= 10.0f;
      } else {
        last_known_good_pose.SetFrom(tracking_state->pose_depth_);
        f_old = f_new;
        for(int i=0; i<36; i++)
          hessian_good[i] = hessian_new[i]/num_valid_points_new;
        for(int i=0; i<6; i++)
          nabla_good[i] = nabla_new[i]/num_valid_points_new;
        lambda /= 10.0f;
      }
      for(int i=0; i<36; i++)
        A[i] = hessian_good[i];
      for(int i=0; i<6; i++)
        A[i+i*6] *= 1.0f+lambda;

      ComputeDelta(step, nabla_good, A, 
                   iteration_type_ != TRACKER_ITERATION_BOTH);
      ApplyDelta(approx_inv_pose, step, approx_inv_pose);
      tracking_state->pose_depth_->SetInverseMatrix(approx_inv_pose);
      tracking_state->pose_depth_->Coerce();
      approx_inv_pose = tracking_state->pose_depth_->GetInverseMatrix();
      if(HasConverged(step))
        break;
    }  
  }
}




#endif /* end of include guard: DEPTH_TRACKER_CPP */
