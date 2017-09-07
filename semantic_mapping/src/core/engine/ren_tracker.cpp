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

}
