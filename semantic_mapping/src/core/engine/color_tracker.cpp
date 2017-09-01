#include "color_tracker.h"

#include <math.h>

using namespace semantic_mapping::engine;

static inline bool MinimizeLM(const ColorTracker& tracker, Pose& init);

ColorTracker::ColorTracker(Vector2i image_size, 
                           TrackerIterationType* tracking_regime,
                           int num_hierarchy_levels, 
                           const LowLevelEngine* low_level_engine,
                           MemoryDeviceType memory_type) {
  view_hierarchy_ = new ImageHierarchy<ViewHierarchyLevel>(image_size,
                    tracking_regime, num_hierarchy_levels, memory_type);
  this->low_level_engine_ = low_level_engine; 
}

ColorTracker::~ColorTracker() {
  delete view_hierarchy_;
}

ColorTracker::TrackCamera(TrackingState* tracking_state, const View* view) {
  this->view_ = view;
  this->tracking_state_ = tracking_state;
  Pose current_pose(
       view->rgbd_calib_->transfer_rgb_to_depth_.calibration_inverse_
       *tracking_state->pose_depth_->GetMatrix());
  
  for(int level = view_hierarchy_->num_levels_-1; level>=0; level--) {
    this->level_id_ = level;
    this->iteration_type_ = view_hierarchy_->levels_[level]->iteration_type_;
    MinimizeLM(*this, current_pose);
  }
  
  tracking_state->pose_depth_->SetMatrix(
           view->rgbd_calib_->transfer_rgb_to_depth_.calibration_ 
           *current_pose.GetMatrix()); 
  tracking_state->pose_depth_->Coerce();
}

void ColorTracker::PrepareForEvaluation(const View* view) {
  low_level_engine_->CopyImage(view_hierarchy_->levels_[0]->rgb_, view->rgb_);
  ImageHierarchy<ViewHierarchyLevel>* hierarchy = view_hierarchy_;
  
  for(int i=1; i<hierarchy->num_levels_; i++) {
    ViewHierarchyLevel* current_level = hierarchy->levels_[i];
    ViewHierarchyLevel* previous_level = hierarchy->levels_[i-1];
    low_level_engine_->FilterSubsample(current_level->rgb_, 
                                       previous_level->rgb_);    
  }

  for(int i=0; i<hierarchy->num_levels_; i++) {
    ViewHierarchyLevel* current_level = hierarchy->levels_[i];
    low_level_engine_->GradientX(current_level->gradient_x_rgb_, 
                                 current_level->rgb_);
    low_level_engine_->GradientY(current_level->gradient_y_rgb_, 
                                 current_level->rgb_);
  }
}

void ColorTracker::ApplyDelta(const Pose& old_pose, const float* delta, 
                              Pose& new_pose) const {
  float pose_vector[6];
  switch(iteration_type_) {
    case TRACKER_ITERATION_ROTATION:
      pose_vector[0] = 0.0f;
      pose_vector[1] = 0.0f;
      pose_vector[2] = 0.0f;
      pose_vector[3] = (float)(delta[0]);
      pose_vector[4] = (float)(delta[1]);
      pose_vector[5] = (float)(delta[2]);
      break;
    case TRACKER_ITERATION_TRANSLATION:
      pose_vector[0] = (float)(delta[0]);
      pose_vector[1] = (float)(delta[1]);
      pose_vector[2] = (float)(delta[2]);
      pose_vector[3] = 0.0f;
      pose_vector[4] = 0.0f;
      pose_vector[5] = 0.0f;
      break;
    case TRACKER_ITERATION_BOTH:
      pose_vector[0] = (float)(delta[0]);
      pose_vector[1] = (float)(delta[1]);
      pose_vector[2] = (float)(delta[2]);
      pose_vector[3] = (float)(delta[3]);
      pose_vector[4] = (float)(delta[4]);
      pose_vector[5] = (float)(delta[5]);
      break;
    default:
      break;
  }
  new_pose.SetFrom(pose_vector);
  new_pose.MultiplyWith(&(old_pose));
}

void ColorTracker::EvaluationPoint::ComputeGradients(bool requires_hessian) {
  int num_para = parent_->NumParameters();
  cache_nabla_ = new float[num_para];
  cache_hessian_ = new float[num_para*num_para];
  parent_->GOneLevel(cache_nabla_, cache_hessian_, pose_); 
}

ColorTracker::EvaluationPoint::EvaluationPoint(Pose* pose, 
                                               ColorTracker* parent) {
  float local_f[1];
  this->pose_ = pose;
  this->parent_ = parent;
  ColorTracker* local_parent = (ColorTracker*) parent_;
  local_parent->FOneLevel(local_f, pose_);
  cache_f_ = local_f;
  cache_hessian_ = NULL;
  cache_nabla_ = NULL; 
}

static inline double StepQuality(ColorTracker::EvaluationPoint* x1, 
                                 ColorTracker::EvaluationPoint* x2,
                                 const float* step, const float* grad,
                                 const float* B, int num_para) {
  double actual_reduction = x1->GetF()-x2->GetF();
  double predicted_reduction = 0.0;
  float* tmp = new float[num_para];
  MatVecMultiple(B, step, tmp, num_para, num_para);
  
  for(int i=0; i<num_para; i++)
    predicted_reduction -= grad[i]*step[i]+0.5*step[i]*tmp[i];
  delete[] tmp;
  
  if(predicted_reduction<0) 
    return actual_reduction/fabs(predicted_reduction);
  return actual_reduction/predicted_reduction;
}

static inline bool MinimizeLM(const ColorTracker& tracker, Pose& init) {
  static const int MAX_STEPS = 100;                                           
  static const float MIN_STEP = 0.00005f;                                     
  static const float MIN_DECREASE = 0.00001f;                                 
  static const float TR_QUALITY_GAMMA1 = 0.75f;                               
  static const float TR_QUALITY_GAMMA2 = 0.25f;                               
  static const float TR_REGION_INCREASE = 2.0f;                               
  static const float TR_REGION_DECREASE = 0.25f;

  int num_para = tracker.NumParameters();
  float* d = new float[num_para];
  float lambda = 0.01f;
  int step_counter = 0;
  
  ColorTracker::EvaluationPoint* x1 = tracker.EvaluateAt(new Pose(init));
  ColorTracker::EvaluationPoint* x2 = NULL;
  
  if(!PortableFinite(x1->GetF())) {
    delete[] d;
    delete x1;
    return false;
  }

  do {
    const float* grad;
    const float* B;
    grad = x1->GetNabla();
    B = x1->GetHessian();
    
    bool success;
    float* A = new float[num_para];
    for(int i=0; i<num_para*num_para; ++i)
      A[i] = B[i];
    for(int i=0; i<num_para; ++i) {
      float& element = A[i*(num_para+1)];
      if(!(fabs(element)<1e-5f))
        element *= (1.0f+lambda);
      else
        element = lambda*1e-10f;
    }
    
    utils::Cholesky cholesky(A, num_para);
    cholesky.BackSub(&(d[0]), grad);
    // TODO: set success to false if cholesky failed
    success = true;
    delete[] A;

    if(success) {
      float max_norm = 0.0;
      for(int i=0; i<num_para; i++) {
        float tmp = fabs(d[i]);
        if(tmp>max_norm)
          max_norm = tmp;
      }
      if(max_norm<MIN_STEP)
        break;
      for(int i=0; i<num_para; i++)
        d[i] = -d[i];

      Pose* tmp_pose = new Pose(x1->GetParameter());
      tracker.ApplyDelta(x1->GetParameter(), &(d[0]), *tmp_pose);

      x2 = tracker.EvaluateAt(tmp_pose);
      double rho = StepQuality(x1, x2, &(d[0]), grad, B, num_para);
      if(rho > TR_QUALITY_GAMMA1)
        lambda = lambda/TR_REGION_INCREASE;
      else if(rho < TR_QUALITY_GAMMA2) {
        success = false;
        lambda = lambda/TR_REGION_DECREASE;
      }
    } else {
      x2 = NULL;
      lambda = lambda/TR_REGION_DECREASE;
    }

    if(success) {
      bool continue_iteration = true;
      if(!(x2->GetF()<(x1->GetF()-fabs(x1->GetF())*MIN_DECREASE)))
        continue_iteration = false;
      delete x1;
      x1 = x2;
      if(!continue_iteration)
        break;
    } else if(x2 != NULL) {
      delete x2;
    }
    
    if(step_counter++ >= MAX_STEPS-1)
      break;
  } while(true);

  init.SetFrom(&(x1->GetParameter()));
  delete x1;
  delete[] d;
  return true;
}


