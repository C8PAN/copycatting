#ifndef RGBD_CALIBRATION_H
#define RGBD_CALIBRATION_H

namespace semantic_mapping {
namespace objects {

class RGBDCalibration {
public:
  Intrinsics instrinsics_rgb_;
  Intrinsics Intrinsics_d_;
  Extrinsics transfer_rgb_to_depth_;
  DisparityCalibration disparity_calibration_;
};
		
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: RGBD_CALIBRATION_H */
