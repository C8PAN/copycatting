#ifndef VIEW_HIERARCHY_LEVEL
#define VIEW_HIERARCHY_LEVEL

namespace semantic_mapping {
namespace objects {
	
class ViewHierarchyLevel {
public:
  int level_id_;
  TrakingIterationType iteration_type_;
  Uchar4Image* rgb_;
  FloatImage* depth_;
  Short4Image* gradient_x_rgb_;
  Short4Image* gradient_y_rgb_;
  Vector4f intrinsics_;
  bool manage_data_;

  ViewHierarchyLevel(Vector2i image_size, int level_id, 
                     TrakingIterationType iteration_type,
                     MemoryDeviceType memory_type,
                     bool skip_allocation) {
    this->manage_data_ = !skip_allocation;
    this->level_id_ = level_id;
    this->iteration_type_ = iteration_type;
    
    if(!skip_allocation) {
      this->rgb_ = new Uchar4Image(image_size, memory_type);
      this->depth_ = new FloatImage(image_size, memory_type);
      this->gradient_x_rgb_ = new Short4Image(image_size, memory_type);
      this->gradient_y_rgb_ = new Short4Image(image_size, memory_type);
    } 
  }

  void UpdateHostFromDevice() {
    this->rgb_->UpdateHostFromDevice();
    this->depth_->UpdateHostFromDevice();
    this->gradient_x_rgb_->UpdateHostFromDevice();
    this->gradient_y_rgb_->UpdateHostFromDevice();
  }

  void UpdateDeviceFromHost() {
    this->rgb_->UpdateDeviceFromHost();
    this->depth_->UpdateDeviceFromHost();
    this->gradient_x_rgb_->UpdateDeviceFromHost();
    this->gradient_y_rgb_->UpdateDeviceFromHost();
  }

  ~ViewHierarchyLevel() {
    if(manage_data_) {
      delete rgb_;
      delete depth_;
      delete gradient_x_rgb_;
      delete gradient_y_rgb_;
    }
  }

  ViewHierarchyLevel(const ViewHierarchyLevel&);
  ViewHierarchyLevel& operator=(const ViewHierarchyLevel&);
};
	
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: VIEW_HIERARCHY_LEVEL */
