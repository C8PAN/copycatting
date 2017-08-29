#ifndef SCENE_H_KJCU9R82
#define SCENE_H_KJCU9R82

#include "scene_parameters.h"

namespace semantic_mapping {
namespace objects {
	
template<class TVoxel, class TIndex>
class Scene {
public:
  bool use_swapping_;
  const SceneParameters* scene_parameters_;
  TIndex index_;
  LocalVoxelBlockArray<TVoxel> local_vba_;
  GlobalCache<TVoxel>* global_cache_;
 
  Scene(const SceneParameters* scene_parameters, bool use_swapping, 
        MemoryDeviceType memory_type) : index_(memory_type), 
        local_vba_(memory_type, index_.GetAllocatedVoxelBlocksNum(), 
                   index_.GetVoxelBlockSize()) {
    this->scene_parameters_ = scene_parameters;
    this->use_swapping_ = use_swapping;
    if(use_swapping)
      global_cache_ = new GlobalCache<TVoxel>();
  }

  ~Scene() {
    if(use_swapping_)
      delete global_cache_;
  }

  Scene(const Scene&);
  Scene& operator=(const Scene&);
};
	
} /*  objects */ 
} /* semantic_mapping */ 



#endif /* end of include guard: SCENE_H_KJCU9R82 */
