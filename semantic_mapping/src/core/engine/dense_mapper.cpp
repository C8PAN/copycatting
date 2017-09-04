#include "dense_mapper.h"

using namespace semantic_mapping::engine;

template<class TVoxel, class TIndex>
DenseMapper<TVoxel, TIndex>::DenseMapper(const Settings* settings) {
  swapping_engine_ = NULL;
  switch(settings->device_type_) {
    case Settings::DEVICE_CPU:
      scene_recon_engine_ = new SceneReconstructionEngine_CPU<TVoxel,TIndex>();
      if(settings->use_swapping_)
        swapping_engine_ = new SwappingEngine_CPU<TVoxel, TIndex>();
      break;
    case Settings::DEVICE_GPU:
      scene_recon_engine_ = new SceneReconstructionEngine_GPU<TVoxel,TIndex>();
      if(settings->use_swapping_)
        swapping_engine_ = new SwappingEngine_GPU<TVoxel, TIndex>();
      break;
    default:
      break;
  }
}

template<class TVoxel, class TIndex>
DenseMapper<TVoxel, TIndex>::~DenseMapper() {
  delete scene_recon_engine_;
  if(swapping_engine_ != NULL)
    delete swapping_engine_;
}

template<class TVoxel, class TIndex>
void DenseMapper<TVoxel, TIndex>::ResetScene(Scene<TVoxel, TIndex>* scene) {
  scene_recon_engine_->ResetScene(scene);
}

template<class TVoxel, class TIndex>
void DenseMapper<TVoxel, TIndex>::ProcessFrame(const View* view, 
                                  const TrackingState* traking_state,
                                  Scene<TVoxel, TIndex>* scene,
                                  RenderState* render_state) {
  scene_recon_engine_->AllocateSceneFromDepth(scene, view, tracking_state, 
                                              render_state);
  scene_recon_engine_->IntegrateIntoScene(scene, view, tracking_state,
                                          render_state);
  if(swapping_engine_ != NULL) {
    swapping_engine_->IntegrateGlobalIntoLocal(scene, render_state);
    swapping_engine_->SaveToGlobalMemory(scene, render_state);
  }
}

template<class TVoxel, class TIndex>
void DenseMapper<TVoxel, TIndex>::UpdateVisibleList(cont View* view, 
                                  const TrackingState* tracking_state,
                                  Scene<TVoxel, TIndex>* scene, 
                                  RenderState* render_state) {
  scene_recon_engine_->AllocateSceneFromDepth(scene, view, tracking_state,
                                              render_state, true);
}

template 
class semantic_mapping::engine::DenseMapper<Voxel, VoxelIndex>;

