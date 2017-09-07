#include "imu_tracker.h"

using namespace semantic_mapping::engine;

ImuTracker::ImuTracker(ImuCalibrator* imu_calib) {
  this->imu_calib_ = imu_calib;
}

ImuTracker::~ImuTracker() { }

void ImuTracker::TrackCamera(TrackingState* tracking_state, const View* view) {
  imu_calib_->RegisterMeasurement(
              ((ViewImu*)view)->imu_measurement_->imu_matrix_);
  tracking_state->pose_depth_->SetRotation(
                  imu_calib_->GetDifferentialRotationChange() * 
                  tracking_state->pose_depth_->GetRoation());
}
