#ifndef MESHING_ENGINE_H
#define MESHING_ENGINE_H

using namespace sementic_mapping::objects;

namespace sementic_mapping {
namespace engine {
	
template<class TVoxel, class TIndex> 
class MeshingEngine {
public:
  virtual void MeshScene(Mesh* mesh, const Scene<TVoxel, TIndex>* scene) = 0;
  MeshingEngine() {}
  virtual ~MeshingEngine() {}
};
	
} /* engine */ 	
} /* sementic_mapping */ 



#endif /* end of include guard: MESHING_ENGINE_H */
