#ifndef SCENE_RECONSTRUCTION_ENGINE_H
#define SCENE_RECONSTRUCTION_ENGINE_H

#include <math.h>

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {

template<class TVoxel, class TIndex>
class SceneReconstructionEngine {
public:
  virtual void ResetScene(Scene<TVoxel, TIndex>* scene) = 0;
  
  virtual void AllocateSceneFromDepth(Scene<TVoxel, TIndex>* scene, 
                                      const View* view, 
                                      const TrackingState* tracking_state,
                                      const RenderState* render_state,
                                      bool only_update_visible_list=false)=0;

  virtual void IntegrateIntoScene(Scene<TVoxel, TIndex>* scene, 
                                  const View* view,
                                  const TrackingState* tracking_state,
                                  const RenderState* render_state) = 0;
  
  SceneReconstructionEngine() {}
  virtual ~SceneReconstructionEngine() {}
};	
	
} /* engine */ 	
} /* semantic_mapping */ 


#endif /* end of include guard: SCENE_RECONSTRUCTION_ENGINE_H */
