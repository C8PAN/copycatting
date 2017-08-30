#ifndef IMU_MEASUREMENT_H
#define IMU_MEASUREMENT_H

namespace semantic_mapping {
namespace objects {
		
class ImuMeasurement {
public:
  Matrix3f imu_matrix_;
  
  ImuMeasurement() {
    this->imu_matrix_.SetIdentity();
  }

  ImuMeasurement(const Matrix3f& imu_matrix) {
    this->imu_matrix_ = imu_matrix;
  }
  
  void SetFrom(const ImuMeasurement* imu_measurement) {
    this->imu_matrix_ = imu_measurement->imu_matrix_;
  }

  ~ImuMeasurement() {}

  ImuMeasurement(const ImuMeasurement);
  ImuMeasurement& operator=(const ImuMeasurement&);
};

} /* objects */ 	
} /* semantic_mapping */ 



#endif /* end of include guard: IMU_MEASUREMENT_H */
