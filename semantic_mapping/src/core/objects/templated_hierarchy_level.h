#ifndef TEMPLATED_HIERARCHY_LEVEL_H
#define TEMPLATED_HIERARCHY_LEVEL_H

namespace semantic_mapping {
namespace objects {
		
template<class ImageType> 
class TemplatedHierarchyLevel {
public:
  int level_id_;
  TrackerIterationType iteration_type_;
  ImageType* depth_;
  Vector4f intrinsics_;
  bool manage_data_;

  TemplatedHierarchyLevel(Vector2i image_size, int level_id, 
                          TrackerIterationType iteration_type,
                          MemoryDeviceType memory_type,
                          bool skip_allocation = false) {
    this->manage_data_ = !skip_allocation;
    this->level_id_ = level_id;
    this->iteration_type_ = iteration_type;
    
    if(!skip_allocation)
      this->depth_ = new ImageType(image_size, memory_type);
  }

  void UpdateHostFromDevice() {
    this->depth_->UpdateHostFromDevice();
  }

  void UpdateDeviceFromHost() {
    this->depth_->UpdateDeviceFromHost();
  }

  ~TemplatedHierarchyLevel() {
    if(manage_data_)
      delete depth_;
  }

  TemplatedHierarchyLevel(const TemplatedHierarchyLevel&);
  TemplatedHierarchyLevel& operator=(const TemplatedHierarchyLevel&);
};

} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: TEMPLATED_HIERARCHY_LEVEL_H */
