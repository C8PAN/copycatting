#ifndef POINT_CLOUD_H_BBSZYUXD
#define POINT_CLOUD_H_BBSZYUXD

#include <stdlib.h>

namespace semantic_mapping {
namespace objects {

class PointCloud {
public:
  uint num_totoal_points_;
  utils::Image<Vector4f>* locations_;
  utils::Image<Vector4f>* colors_;

  explicit PointCloud(Vector2i image_size, MemoryDeviceType memory_type) {
    this->num_totoal_points_ = 0;
    locations_ = new utils::Image<Vector4f>(image_size, memory_type);
    colors_ = new utils::Image<Vector4f>(image_size, memory_type);
  }

  void UpdateHostFromDevice() {
    this->locations_->UpdateHostFromDevice();
    this->colors_->UpdateHostFromDevice();
  }

  void UpdateDeviceFromHost() {
    this->locations_->UpdateDeviceFromHost;
    this->color_->UpdateDeviceFromHost();
  }
 
  ~PointCloud() {
    delete locations_;
    delete color_;
  }

  PointCloud(const PointCloud&);
  PointCloud& operator=(const PointCloud&);
};	
	
} /* objects */ 	
} /* semantic_mapping */ 



#endif /* end of include guard: POINT_CLOUD_H_BBSZYUXD */
