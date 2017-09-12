#include "ren_tracker.h"

#include <math.h>

using namespace semantic_mapping::engine;

static inline void GetRotationMatrixFromMRP(float* outR, const float* r) {
  float t1 = r[0], t2 = r[1], t3 = r[2];                                      
  float tsq = t1*t1 + t2*t2 + t3*t3;                                          
  float tsum = 1 - tsq;                                                       
  outR[0] = 4 * t1*t1 - 4 * t2*t2 - 4 * t3*t3 + tsum*tsum;    
  outR[1] = 8 * t1*t2 - 4 * t3*tsum;  
  outR[2] = 8 * t1*t3 + 4 * t2*tsum;
  outR[3] = 8 * t1*t2 + 4 * t3*tsum;  
  outR[4] = 4 * t2*t2 - 4 * t1*t1 - 4 * t3*t3 + tsum*tsum;    
  outR[5] = 8 * t2*t3 - 4 * t1*tsum;
  outR[6] = 8 * t1*t3 - 4 * t2*tsum;  
  outR[7] = 8 * t2*t3 + 4 * t1*tsum;  
  outR[8] = 4 * t3*t3 - 4 * t2*t2 - 4 * t1*t1 + tsum*tsum;
  for(int i = 0; i<9; i++) 
    outR[i] /= ((1 + tsq)*(1 + tsq)); 
}

static inline void GetMFromParam(float* pose, Matrix4f& M) {                                                                               
  float R[9];                                                                 
  GetRotationMatrixFromMRP(R, &(pose[3]));                                    
                                                                                
  M.m00 = R[0]; M.m01 = R[3]; M.m02 = R[6]; M.m03 = 0;                        
  M.m10 = R[1]; M.m11 = R[4]; M.m12 = R[7]; M.m13 = 0;                        
  M.m20 = R[2]; M.m21 = R[5]; M.m22 = R[8]; M.m23 = 0;                        
  M.m30 = pose[0]; M.m31 = pose[1]; M.m32 = pose[2]; M.m33 = 1;               
}                                                                               


static void ComputeSingleStep(float *step, float *ATA, float *ATb, 
                              float lambda) {
  float tmpATA[6 * 6];                                                        
  memcpy(tmpATA, ATA, 6 * 6 * sizeof(float));                                 
  memset(step, 0, 6 * sizeof(float));                                         
                                                                                 
  for(int i = 0; i < 6 * 6; i += 7) {                                         
    float &ele = tmpATA[i];                                                 
    if(!(fabs(ele) < 1e-15f)) 
      ele *= (1.0f + lambda); 
    else 
      ele = lambda*1e-10f;
  }                                                                           
  utils::Cholesky cholA(tmpATA, 6);                                         
  cholA.BackSub(step, ATb);                                                   
  for(int i = 0; i < 6; i++) 
    step[i] = -step[i];                             
}   

template<class TVoxel, class TIndex>
RenTracker<TVoxel, TIndex>::RenTracker(Vector2i image_size, 
                                       TrackerIterationType* tracking_regime,
                                       int num_hierarchy_levels,
                                       const LowLevelEngine* low_level_engine,
                                       const Scene<TVoxel, TIndex>* scene,
                                       MemoryDeviceType memory_type) {
  view_hierarchy_ = new ImageHierarchy<TemplatedHierarchyLevel<Float4Image> >(
                    image_size, tracking_regime, num_hierarchy_levels, 
                    memory_type, false);
  image1_ = new FloatImage(image_size, memory_type);
  image2_ = new FloatImage(image_size, memory_type);
  this->low_level_engine_ = low_level_engine;
  this->scene_ = scene;
}

template<class TVoxel, class TIndex>
RenTracker<TVoxel, TIndex>::~RenTracker() {
  delete image1_;
  delete image2_;
  delete view_hierarchy_;
}

template<class TVoxel, class TIndex>
void RenTracker<TVoxel, TIndex>::PrepareForEvaluation(const View* view) {
  Vector4f intrinsics 
    = view->rgbd_calib_->intrinsics_depth_.ProjectionParameters.all;
  this->image1_->num_dims_ = view->depth_->num_dims_;
  this->image1_->data_size_ = view->depth_->data_size_;
  low_level_engine_->CopyImage(this->image1_, view->depth_);
  view_hierarchy_->levels_[0]->intrinsics_ = intrinsics;
  UnprojectDepthToCamera(view->depth_, view_hierarchy_->levels_[0]->depth_, 
                         intrinsics);
  
  for(int i=1; i<view_hierarchy_->num_levels_; i++) {
    TemplatedHierarchyLevel<Float4Image>* current_level_view 
      = view_hierarchy_->levels_[i]
    TemplatedHierarchyLevel<Float4Image>* previous_level_view 
      = view_hierarchy->levels[i-1];
    low_level_engine_->FilterSubsampleWithHoles(image2_, image1_);
    current_level_view->intrinsics_ = previous_level_view->intrinsics_*0.5f;
    UnprojectDepthToCamera(image2_, view_hierarchy_->levels_[i]->depth_,
                           view_hierarchy_->levels_[i]->intrinsics_);
    this->image1_->num_dims_ = this->image2_->num_dims_;
    this->image1_->data_size_ = this->image2_->data_size_;
    low_level_engine_->CopyImage(this->image1_, this->image2_);
  } 
}

template<class TVoxel, class TIndex>
void RenTracker<TVoxel, TIndex>::TrackCamera(TrackingState* tracking_state,
                                             const View* view) {
  this->PrepareForEvaluation(view);
  float last_energy = 0.0f;
  float current_energy = 0.0f;
  float lambda = 1000.0f;
  float step[6];
  Matrix4f invM, tmpM;
  bool converged = false;
  int iter;
  this->level_id_ = 0;
  invM = tracking_state->pose_depth_->GetInverseMatrix();
  static const int MAX_STEPS = 100;
  static const float MIN_STEP = 0.00005f;
  static const float MIN_DECREASE = 0.0001f;
  static const float TR_REGION_INCREASE = 0.10f;
  static const float TR_REGION_DECREASE = 10.0f;
 
  FOneLevel(&last_energy, invM);
  for(iter=0; iter<MAX_STEPS; iter++) {
    GOneLevel(nabla_, hessian_, invM);
    while(true) {
      ComputeSingleStep(step, hessian_, nabla_, lambda);
      float max_norm = 0.0f;
      for(int i=0; i<6; i++) {
        float tmp = fabs(step[i]);
        if(tmp>max_norm)
          max_norm = tmp;
      }
      if(max_norm<MIN_STEP) {
        converged = true;
        break;
      }
      GetMFromParam(step, tmpM);
      FOneLevel(&current_energy, tmpM*invM);
      if(current_energy < last_energy) {
        if(fabs(current_energy-last_energy)/fabs(last_energy) < MIN_DECREASE) 
          converged = true;
        last_energy = current_energy;
        lambda *= TR_REGION_INCREASE;
        invM = tmpM*invM;
        break;
      } else {
        lambda *= TR_REGION_DECREASE;
      }
    }
    if(converged)
      break;
  }
  tracking_state->pose_depth_->SetInverseMatrix(invM);
  tracking_state->pose_depth_->Coerce(); 
}

template<class TVoxel, class TIndex>
void RenTracker<TVoxel, TIndex>::ApplyDelta(const Pose& old_pose, 
                                 const float* delta, Pose& new_pose) const {
  float R[9];
  Matrix4f deltaM;
  GetRotationMatrixFromMRP(R, &(delta[3]));
  deltaM.m00 = R[0];
  deltaM.m01 = R[3];
  deltaM.m02 = R[6];
  deltaM.m03 = 0.0f;
  deltaM.m10 = R[1];
  deltaM.m11 = R[4];
  deltaM.m12 = R[7];
  deltaM.m13 = 0.0f;
  deltaM.m20 = R[2];
  deltaM.m21 = R[5];
  deltaM.m22 = R[8];
  deltaM.m23 = 0.0f;
  deltaM.m30 = delta[0];
  deltaM.m31 = delta[1];
  deltaM.m32 = delta[2];
  deltaM.m33 = 1.0f;

  new_pose.SetInverseMatrix(deltaM*old_pose.GetInverseMatrix());
  new_pose.Coerce();
}

// !!!
template<class TVoxel, class TIndex>
static inline double StepQuality(
                     typename RenTracker<TVoxel, TIndex>::EvaluationPoint* x1, 
                     typename RenTracker<TVoxel, TIndex>::EvaluationPoint* x2, 
                     const float* step, const float* grad, const float* B,
                     int num_param) {
  double actual_reduction = x1->GetF() - x2->GetF();
  double predicted_reduction = 0.0;
  float* tmp = new float[num_param];
  MatVecMultiple(B, step, tmp, num_param, num_param);
  for(int i=0; i<num_param; i++)
    predicted_reduction -= grad[i]*step[i]+0.5*step[i]*tmp[i];
  delete[] tmp;
  if(predicted_reduction<0)
    return actual_reduction/fabs(predicted_reduction);
  return actual_reduction/predicted_reduction;
}

template<class TVoxel, class TIndex>
static inline bool MinimizeLM(const RenTracker<TVoxel, TIndex>& tracker, 
                              Pose& init) {
  static const int MAX_STEPS = 100;
  static const float MIN_STEP = 0.00005f;
  static const float MIN_DECREASE = 0.00001f;
  static const float TR_QUALITY_GAMMA1 = 0.75f;
  static const float TR_QUALITY_GAMMA2 = 0.25f;
  static const float TR_REGION_INCREASE = 2.0f;
  static const float TR_REGION_DECREASE = 0.3f;
 
  int num_param = tracker.NumParameters();
  float* d = new float[num_param];
  float lambda = 1000.0f;
  int step_counter = 0;
  
  typename RenTracker<TVoxel, TIndex>::EvaluationPoint* x1 
           = tracker.EvaluateAt(new Pose(init));
  typename RenTracker<TVoxel, TIndex>::EvaluationPoint* x2 = NULL;
  if(!PortableFinite(x1-GetF())) {
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
    
    float* A = new float[num_param*num_param];
    for(int i=0; i<num_param*num_param; i++)
      A[i] = B[i];
    for(int i=0; i<num_param; i++) {
      float& element = A[i*(num_param+1)];
      if(!(fabs(element)<1e-5f))
        element *= (1.0f+lambda);
      else
        element = lambda*1e-10f;
    }
    utils::Cholesky cholesky(A, num_param);
    cholesky.BackSub(&(d[0]), grad);
    success = true;
    delete[] A;

    if(success) {
      float max_norm = 0.0;
      for(int i=0; i<num_param; i++) {
        float tmp = fabs(d[i]);
        if(tmp>max_norm)
          max_norm = tmp;
      }
      if(max_norm < MIN_STEP)
        break;
      for(int i=0; i<num_param; i++)
        d[i] = -d[i];
      Pose* tmp_param = new Pose(x1->GetMFromParam());
      tracker.ApplyDelta(x1->GetParameter(), &(d[0]), *tmp_param);
      x2 = tracker.EvaluateAt(tmp_param);
      double rho = StepQuality<TVoxel, TIndex>(x1, x2, &(d[0]), grad, B, 
                                               num_param);
      if(rho > TR_QUALITY_GAMMA1) {
        lambda = lambda/TR_REGION_INCREASE;
      } else if(rho <= TR_QUALITY_GAMMA2) {
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

template class semantic_mapping::engine::RenTracker<Voxel, VoxelIndex>;

