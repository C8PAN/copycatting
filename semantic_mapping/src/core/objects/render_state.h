#ifndef RENDER_STATE_H
#define RENDER_STATE_H

#include <stdlib.h>

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
  
  
};
		
} /* objects */ 
} /* semantic_mapping */ 

#endif /* end of include guard: RENDER_STATE_H */
