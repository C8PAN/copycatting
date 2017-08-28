#ifndef EXTRINSICS_H
#define EXTRINSICS_H

#include <stdio.h>

namespace semantic_mapping {
namespace objects {

class Extrinsics {
public:
  Matrix4f calibration_;
  Matrix4f calibration_inverse_;
  
  void SetFrom(const Matrix4f& src) {
    this->calibration_ = src;
    this->calibration_inverse_.SetIdentity();
    
    for(int r=0; r<3; ++r)
      for(int c=0; c<3; ++c)
        this->calibration_inverse_.m[r+4*c] = this->calibration_.m[c+4*r];

    for(int r=0; r<3; ++r) {
      float& dest = this->calibration_inverse_.m[r+4*3];
      dest = 0.0f;
      for(int c=0; c<3; ++c)
        dest -= this->calibration_.m[c+4*r]*this->calibration_.m[c+4*3];
    }
  }

  Extrinsics() {
    Matrix4f m;
    m.SetZeros();
    m.m00 = m.m11 = m.m22 = m.m33 = 1.0;
    SetFrom(m);
  }
 
};
		
} /* objects */ 
} /* semantic_mapping */ 



#endif /* end of include guard: EXTRINSICS_H */
