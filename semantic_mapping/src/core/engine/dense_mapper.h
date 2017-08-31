#ifndef DENSE_MAPPER_H
#define DENSE_MAPPER_H

namespace semantic_mapping {
namespace engine {

template<class TVoxel, class TIndex>
class DenseMapper {
public:
  void ResetScene(Scene<TVoxel, TIndex>* scene);
  
  void ProcessFrame(const View* view, 
                    const TrackingState* tracking_state, 
                    Scene<TVoxel, TIndex>* scene, 
                    RanderState* render_state_live);

  void UpdateVisibleList(const View* view, 
                         const TrackingState* tracking_state,
                         Scene<TVoxel, TIndex>* scene, 
                         RenderState* render_state);

  explicit DenseMapper(const Settings* settings);
  
  ~DenseMapper();   

private:
  SceneReconstructionEngine<TVoxel, TIndex>* scene_recon_engine_;
  SwappingEngine<TVoxel, TIndex>* swapping_engine_;
};	
	
} /* engine */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: DENSE_MAPPER_H */
