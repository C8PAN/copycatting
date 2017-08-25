#ifndef INTRINSICS_H
#define INTRINSICS_H

#include <string>

namespace semantic_mapping {
namespace objects {

class Intrinsics {
public:
  struct ProjectionParameters {
    Vector4f all;
    float fx, fy, cx, cy; 
  } projection_parameters_;

  void SetFrom(float fx, float fy, float cx, float cy, 
               float size_x, float size_y) {
    projection_parameters_.fx = fx;
    projection_parameters_.fy = fy;
    projection_parameters_.cx = cx;
    projection_parameters_.cy = cy;
    projection_parameters_.all.x = fx;
    projection_parameters_.all.y = fy;
    projection_parameters_.all.z = cx;
    projection_parameters_.all.w = cy;
  }

  Intrinsics() {
    // for standard kinect RGB camera
    SetFrom(580, 580, 320, 240, 640, 480);
  }
};
		
} /* objects */ 
} /* semantic_mapping */ 


#endif /* end of include guard: INTRINSICS_H */
