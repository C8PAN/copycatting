#include "tracking_controller.h"

using namespace semantic_mapping::engine;

void TrackingController::Track(TrakingState* tracking_state, const View* view) {
  if(tracking_state->age_point_cloud_ != 1)
    tracking_state->requires_full_rendering_ 
      = (tracking_state->TrackerFarFromPointCloud 
         || !settings_->use_approximate_raycast_);
}

void TrackingController::Prepare(TrackingState* tracking_state, View* view,
                                 RenderState* render_state) {
  if(settings_->tracker_type_ == Settings::TRACKER_COLOR) {
    Pose rgb_pose(view->rgbd_calib_->transfer_rgb_to_depth_.calibration_inverse_
                  *tracking_state->pose_depth_->GetMatrix());
    visualisation_engine_->CreateExpectedDepths(&rgb_pose, 
                  &(view->rgbd_calib_->instrinsics_rgb_), render_state);
    visualisation_engine_->CreatePointCloud(view, tracking_state, 
                  render_state, settings_->skip_points_);
    tracking_state->age_point_cloud_ = 0; 
  } else {
    visualisation_engine_->CreateExpectedDepths(traking_state->pose_depth_,
                  &(view->rgbd_calib_->instrinsics_depth_), render_state);
    
    if(traking_state->requires_full_rendering_) {
      visualisation_engine_->CreateIcpMap(view, tracking_state, render_state);
      tracking_state->pose_point_cloud_->SetFrom(traking_state->pose_depth_); 
      if(tracking_state->age_point_cloud_ == -1)
        tracking_state->age_point_cloud_ = -2;
      else
        tracking_state->age_point_cloud_ = 0;
    } else {
      visualisation_engine_->ForwardRender(view, tracking_state, render_state);
      tracking_state->age_point_cloud_++; 
    }   
  }
}


