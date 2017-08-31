#ifndef VIEW_IMU_H
#define VIEW_IMU_H

namespace semantic_mapping {
namespace objects {
		
class ViewImu : public View {
public:
  ImuMeasurement* imu_measurement_;
  
  ViewImu(const RgbdCalibration* calibration, Vector2i image_size_rgb, 
          Vector2i image_size_depth, bool use_gpu) 
          : View(calibration, image_size_rgb, image_size_depth, use_gpu) {
    imu_measurement_ = new ImuMeasurement()
  }

  ~ViewImu() { delete imu_measurement_; }

  ViewImu(const ViewImu&);
  ViewImu& operator=(const ViewImu&);
};

} /* objects */ 		
} /* semantic_mapping */ 


#endif /* end of include guard: VIEW_IMU_H */
