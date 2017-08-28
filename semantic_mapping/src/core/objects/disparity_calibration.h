#ifndef DISPARITY_CALIBRATION_H
#define DISPARITY_CALIBRATION_H

#include <stdlib.h>

namespace semantic_mapping {
namespace objects {

class DisparityCalibration {
public:
  typedef enum { 
    TRANSFER_KINECT, TRANSFER_AFFINE 
  } TransferType;
  
  TransferType transfer_type_;
  Vector2f parameters_;
  void SetFrom(float a, float b, TransferType type) {
    parameters_.x = a;
    parameters_.y = b;
    transfer_type_ = type;
  }

  DisparityCalibration() {
    parameters_.x = 1.0f/1000.0f;
    parameters_.y = 0.0f;
    transfer_type_ = TRANSFER_AFFINE;
  }
};
		
} /* objects */ 
} /* semantic_mapping */ 



#endif /* end of include guard: DISPARITY_CALIBRATION_H */
