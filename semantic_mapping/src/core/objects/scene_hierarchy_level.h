#ifndef SCENE_HIERARCHY_LEVEL_H
#define SCENE_HIERARCHY_LEVEL_H

namespace semantic_mapping {
namespace objects {
	
class SceneHierarchyLevel {
public:
  int level_id_;
  TrackerIterationType iteration_type_;
  Float4Image* points_map_;
  Float4Image* normals_map_;
  Vector4f intrinsics_;
  bool manage_data_;

  SceneHierarchyLevel(Vectro2i image_size, int level_id, 
                      TrackerIterationType iteration_type, 
                      MemoryDeviceType memory_type, 
                      bool skip_allocation = false) {
    this->manage_data_ = !skip_allocation;
    this->level_id_ = level_id;
    this->iteration_type_ = iteration_type;
    
    if(!skip_allocation) {
      this->points_map_ = new Float4Image(image_size, memory_type);
      this->normals_map_ = new Float4Image(image_size, memory_type);
    } 
  }

  void UpdateHostFromDevice() {
    this->points_map_->UpdateHostFromDevice();
    this->normals_map_->UpdateHostFromDevice();
  }

  void UpdateDeviceFromHost() {
    this->points_map_->UpdateDeviceFromHost();
    this->normals_map_->UpdateDeviceFromHost();
  }

  ~SceneHierarchyLevel() {
    if(manage_data_) {
      delete points_map_;
      delete normals_map_;
    }
  }

  SceneHierarchyLevel(const SceneHierarchyLevel&);
  SceneHierarchyLevel& operator=(const SceneHierarchyLevel&);
};
	
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: SCENE_HIERARCHY_LEVEL_H */
