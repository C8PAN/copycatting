#ifndef SWAPPING_ENGINE_H
#define SWAPPING_ENGINE_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace objects {
	
template<class TVoxel, class TIndex> 
class SwappingEngine {
public:
  virtual void IntegrateGlobalIntoLocal(Scene<TVoxel, TIndex>* scene, 
                                        RenderState* render_state) = 0;

  virtual void SaveToGlobalMemory(Scene<TVoxel, TIndex>* scene, 
                                  RenderState* render_state) = 0;

  virtual ~SwappingEngine(0 {}
};
	
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: SWAPPING_ENGINE_H */
