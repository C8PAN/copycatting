#include "visualisation_engine.h"

using namespace semantic_mapping::engine;

inline float interpolate(float val, float y0, float x0, float y1, float x1) {
  return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

inline float base(float val) {
  if(val <= -0.75f) 
    return 0.0f;
  else if(val <= -0.25f)
    return interpolate(val, 0.0f, -0.75f, 1.0f, -0.25f);
  else if(val <= 0.25f)
    return 1.0f;
  else if(val <= 0.75f)
    return interpolate(val, 1.0f, 0.25f, 0.0f, 0.75f);
  else 
    return 0.0f;
}

void VisualisationEngine::DepthToUchar4(Uchar4Image* dst, FloatImage* src) {
  Vector4u* dest = dst->GetData(MEMORY_DEVICE_CPU);
  float* source = src->GetData(MEMORY_DEVICE_CPU);
  int data_size = static_cast<int>(dst->data_size_);
  memset(dst->GetData(MEMORY_DEVICE_CPU), 0 , data_size*4);
  Vector4u* dest_uc4;
  float lims[2], scale;
  dest_uc4 = (Vector4u*)dest;
  lims[0] = 100000.0f;
  lims[1] = -100000.0f;
  
  for(int idx=0; idx<data_size; i++) {
    float source_val = source[idx];
    if(source_val>0.0f) {
      lims[0] = MIN(lims[0], source_val);
      lims[1] = MAX(lims[1], source_val);
    }
  }    

  scale = ((lims[1]-lims[0]) != 0) ? 1.0f/(lims[1]-lims[0]) : 1.0f/lims[1];
  
  if(lims[0] == lims[1])
    return;
  
  for(int idx=0; idx<data_size; idx++) {
    float source_val = source[idx];
    if(source_val>0.0f) {
      source_val = (source_val-lims[0])*scale;
      dest_uc4[idx].r = (uchar)(base(source_val-0.5f)*255.0f);
      dest_uc4[idx].g = (uchar)(base(source_val)*255.0f);
      dest_uc4[idx].b = (uchar)(base(source_val+0.5f)*255.0f);
      dest_uc4[idx].a = 255; 
    }
  }
}


// 貌似有问题
void VisualisationEngine::NormalToUchar4(Uchar4Image* dst, Float4Image* src) {
  Vector4u* dest = dst->GetData(MEMORY_DEVICE_CPU);
  Vector4f* source = src->GetData(MEMORY_DEVICE_CPU);
  int data_size = static_cast<int>(dst->data_size_);
  memset(dst->GetData(MEMORY_DEVICE_CPU), 0 data_size*4);

  for(int idx=0; idx<data_size; idx++) {
    Vector4f source_val = source[idx];
    if(source_val.w>=0.0f) {
      dest[idx].r = (uchar)((0.3f+(source_val.r+1.0f)*0.35f)*255.0f);
      dest[idx].g = (uchar)((0.3f+(source_val.g+1.0f)*0.35f)*255.0f);
      dest[idx].b = (uchar)((0.3f+(source_val.b+1.0f)*0.35f)*255.0f);
    }
  }
}

void VisualisationEngine::WeightToUchar4(Uchar4Image* dst, FloatImage* src) {
  Vector4u* dest = dst->GetData(MEMORY_DEVICE_CPU);
  float* source = src->GetData(MEMORY_DEVICE_CPU);
  int data_size = static_cast<int>(dst->data_size_);

  float min_depth = 1000;
  for(size_t i=0; i<src->data_size_; i++) {
    if(source[i]>0)
      min_depth = MIN(min_depth, source[i]);
  }

  memset(dst->GetData(MEMORY_DEVICE_CPU), 0, data_size*4);

  for(int idx=0; idx<data_size; idx++) {
    float source_val = source[idx];
    if(source_val>0) {
      source_val = min_depth/source_val*0.8f +0.2;
      dest[idx].r = (uchar)((1-source_val)*255.0f);
      dest[idx].g = (uchar)(source_val*255.0f)
      dest[idx].b = 0.0f;
    }
  }
}

template 
class semantic_mapping::engine::VisualisationEngine<Voxel, VoxelIndex>;


