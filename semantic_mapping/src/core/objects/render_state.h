#ifndef RENDER_STATE_H
#define RENDER_STATE_H

#include <stdlib.h>
#include "defines.h"


namespace semantic_mapping {
namespace objects {

class RenderState {
public:
  utils::Image<Vector2f> *rendering_range_image_;
  utils::Image<Vector4f> *raycast_result_;
  utils::Image<Vector4f> *forward_projection_;
  utils::Image<int> *forward_projection_missing_points_;
  int num_forward_projection_missing_points_;
  utils::Image<Vector4u> *raycast_image_;
  
  RenderState(const Vector2i& image_size, float vf_min, float vf_max, 
              MemoryDevice memory_type) {
    rendering_range_image_ = new utils::Image<Vector2f>(image_size, memory_type);
    raycast_result_ = new utils::Image<Vector4f>(image_size, memory_type);
    forward_projection_ = new utils::Image<Vector4f>(image_size, memory_type);
    forward_projection_missing_points_ = 
      new utils::Image<int>(image_size, memory_type);
    raycast_image_ = new utils::Image<Vector4u>(image_size, memory_type);
    utils::Image<Vector2f>* buff_image = 
      new utils::Image<Vector2f>(image_size, MEMORY_DEVICE_CPU);
    
    Vector2f v_lims(vf_min, vf_max);
    for(int i=0; i<image_size.x*image_size.y; i++)
      buff_image->GetData(MEMORY_DEVICE_CPU)[I] = v_lims;
    
    if(memory_type == MEMORY_DEVICE_GPU)
      rendering_range_image_->SetFrom(buff_image, 
                              utils::MemoryBlock<Vector2f>::CPU_TO_GPU); 
    else
      rendering_range_image_->SetFrom(buff_image,
                              utils::MemoryBlock<Vector2f>::CPU_TO_CPU);

    delete buff_image;
    num_forward_projection_missing_points_ = 0; 
  }

  virtual ~RenderState() {
    delete rendering_range_image_;
    delete raycast_image_;
    delete forward_projection_;
    delete forward_projection_missing_points_;
    delete raycast_image_;
  }

};
		
} /* objects */ 
} /* semantic_mapping */ 

#endif /* end of include guard: RENDER_STATE_H */
