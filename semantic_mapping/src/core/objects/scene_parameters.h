#ifndef SCENE_PARAMETERS_H
#define SCENE_PARAMETERS_H

namespace semantic_mapping {
namespace objects {

class SceneParameters {
public:
  float voxel_size_;
  float view_frustum_min_, view_frustum_max_;
  float mu_;
  int max_observations_;
  bool stop_intergrating_at_max_observations_;
  
  SceneParameters(float mu, float max_observations, float voxel_size,
                  float view_frustum_min, float view_frustum_max,
                  bool stop_intergrating_at_max_observations) {
    this->mu_ = mu;
    this->max_observations_ = max_observations;
    this->voxel_size_ = voxel_size;
    this->view_frustum_min_ = view_frustum_min;
    this->view_frustum_max_ = view_frustum_max;
    this->stop_intergrating_at_max_observations_ 
          = stop_intergrating_at_max_observations;
  } 

  explicit SceneParameters(const SceneParameters *scene_parameters) {
    this->SetFrom(scene_parameters);
  }

  void SetFrom(const SceneParameters* scene_parameters) {
    this->voxel_size_ = scene_parameters->voxel_size_;
    this->view_frustum_min_ = scene_parameters->view_frustum_min_;
    this->view_frustum_max_ = scene_parameters->view_frustum_max_;
    this->mu_ = scene_parameters->mu_;
    this->max_observations_ = scene_parameters->max_observations_;
    this->stop_intergrating_at_max_observations_ 
          = scene_parameters->stop_intergrating_at_max_observations_;
  } 
};
		
} /* objects */ 		
} /* semantic_mapping */ 


#endif /* end of include guard: SCENE_PARAMETERS_H */
