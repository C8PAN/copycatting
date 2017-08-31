#ifndef IMAGE_HIERARCHY_H
#define IMAGE_HIERARCHY_H

namespace semantic_mapping {
namespace  objects {
	
template<class T> 
class ImageHierarchy {
public:
  int num_levels_;
  T** levels_;
  
  ImageHierarchy(Vector2i image_size, TrackerIterationType* traking_regime, 
                 int num_hierarchy_levels, MemoryDeviceType memory_type,
                 bool skip_allocation_for_level0 = false) {
    this->num_levels_ = num_hierarchy_levels;
    levels_ = new T*[num_hierarchy_levels];
    for(int i=num_hierarchy_levels-1; i>=0; i--) 
      levels_[i] = new T(image_size, i, traking_regime[i], memory_type, 
                         (i==0) && skip_allocation_for_level0);
  }

  void UpdateHostFromDevice() {
    for(int i=0; i<num_levels_; i++)
      this->levels_[i]->UpdateHostFromDevice();
  }

  void UpdateDeviceFromHost() {
    for(int i=0; i<num_levels_; i++)
      this->levels_[i]->UpdateDeviceFromHost();
  }

  ~ImageHierarchy() {
    for(int i=0; i<num_levels_; i++)
      delete levels_[i];
    delete[] levels_;
  }

  ImageHierarchy(const ImageHierarchy&);
  ImageHierarchy& operator=(const ImageHierarchy&);
};
	
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: IMAGE_HIERARCHY_H */
