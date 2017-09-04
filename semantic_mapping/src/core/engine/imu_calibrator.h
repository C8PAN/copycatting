#ifndef IMU_CALIBRATOR_H
#define IMU_CALIBRATOR_H

namespace semantic_mapping {
namespace objects {
	
class ImuCalibrator {
public:
  virtual void RegisterMeasurement(const Matrix3f& R) = 0;
  virtual Matrix3f GetDifferentialRotationChange() = 0;
  ImuCalibrator() {}
  virtual ~ImuCalibrator() {}

  ImuCalibrator(const ImuCalibrator&); 
  ImuCalibrator& operator=(const ImuCalibrator&);
};

class ImuCalibrator_iPad : public ImuCalibrator {
public:
  void RegisterMeasurement(const Matrix3f& R) {
    oldR_imu_ = imu_coords_->GetRoation();
    imu_coords_->SetRotation(R);
    imu_coords_->GetParameters(t_imu_, r_imu_);
    imu_coords_->SetFrom(t_imu_, -r_imu_);
    newR_imu_ = imu_coords_->GetRoation();   
  }  

  Matrix3f GetDifferentialRotationChange() {
    if(has_two_frames_) {
      oldR_imu_.Inverse(inv_oldR_imu);
      camera_coords_->SetRotation(imu_coords_->GetRoation()*inv_oldR_imu);
      camera_coords_->GetParameters(t_imu_, r_imu_);
      camera_coords_->SetFrom(t_imu_.x, t_imu_.y, t_imu_.z, 
                              -r_imu_.y, -r_imu_.x, -r_imu_.z);
    }
    has_two_frames_ = true;
    return camera_coords_->GetRoation();
  }

  ImuCalibrator_iPad : ImuCalibrator() {
    has_two_frames_ = false;
    imu_coords_ = new Pose();
    imu_coords_->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    camera_coords_ = new Pose();
    camera_coords_->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    oldR_imu_.SetIdentity();
  }

  ~ImuCalibrator_iPad() {
    delete imu_coords_;
    delete camera_coords_;
  }

private:
  Pose* imu_coords_;
  Pose* camera_coords_;
  Vector3f t_imu_;
  Vector3f r_imu_;
  Matrix3f newR_imu_;
  Matrix3f oldR_imu_;
  bool has_two_frames_;
};
	
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: IMU_CALIBRATOR_H */
