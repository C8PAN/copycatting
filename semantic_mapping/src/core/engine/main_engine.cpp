#include "main_engine.h"

using namespace semantic_mapping::engine;

MainEngine::MainEngine(const Settings* settings, const RgbdCalibration* calib,
                       Vector2i image_size_rgb, Vector2i image_size_d) {
  static const bool create_meshing_engine = true;
  if((image_size_rgb.x==-1) || (image_size_rgb.y==-1))
    image_size_d = image_size_rgb;
  this->settings_ = settings;
  this->scene_ = new Scene<Voxel, VoxelIndex>(&(settings_->scene_parameters_),
                 settings_->use_swapping_, settings_->device_type_ == 
                 Settings::DEVICE_GPU ? MEMORY_DEVICE_GPU : MEMORY_DEVICE_CPU);
  meshing_engine_ = NULL;
  
  switch(settings_->device_type_) {
    case Settings::DEVICE_CPU:
      low_level_engine_ = new LowLevelEngine_CPU();
      view_builder_ = new ViewBuilder_CPU(calib);
      visualisation_engine_ 
        = new VisualisationEngine_CPU<Voxel, VoxelIndex>(scene);
      if(create_meshing_engine)
        meshing_engine_ = new MeshingEngine_CPU<Voxel, VoxelIndex>();
      break;
    case Settings::DEVICE_GPU:
      low_level_engine_ = new LowLevelEngine_GPU();
      view_builder_ = new ViewBuilder_GPU(calib);
      visualisation_engine_ 
        = new VisualisationEngine_GPU<Voxel, VoxelIndex>(scene);
      if(create_meshing_engine)
        meshing_engine_ = new MeshingEngine_GPU<Voxel, VoxelIndex>();
      break;
    default:
      break;
  }
  
  mesh_ = NULL;
  if(create_meshing_engine)
    mesh_ = new Mesh(settings_->device_type_ == Settings::DEVICE_GPU ? 
            MEMORY_DEVICE_GPU : MEMORY_DEVICE_CPU);
  Vector2i tracked_image_size = TrackingController::GetTrackedImageSize(
                                settings_, image_size_rgb, image_size_d);
  render_state_live_ 
    = VisualisationEngine->CreateRenderState(tracked_image_size);
  render_state_free_view_ = NULL;
  
  dense_mapper_ = new DenseMapper<Voxel, VoxelIndex>(settings_);
  dense_mapper_->ResetScene(scene_);
  imu_calibrator_ = new ImuCalibrator_iPad();
  tracker_ = TrackerFactory<Voxel, VoxelIndex>::Instance().Make(
             tracked_image_size, settings_, low_level_engine_, imu_calibrator_,
             scene_);
  tracking_controller_ = new TrackingController(tracker_,visualisation_engine_,
                         low_level_engine_, settings_);
  tracking_state_=tracking_controller_->BuildTrackingState(tracked_image_size);  
  tracker_->UpdateInitialPose(tracking_state_);
  view_ = NULL;
  fusion_active_ = true;
  main_processing_active_ = true; 
}

MainEngine::~MainEngine() {
  delete render_state_live_;
  if(render_state_free_view_ != NULL)
    delete render_state_free_view_;
  delete scene_;
  delete dense_mapper_;
  delete tracking_controller_;
  delete tracker_;
  delete imu_calibrator_;
  delete low_level_engine_;
  delete view_builder_;
  delete tracking_state_;
  if(view_ != NULL)
    delete view_;
  delete visualisation_engine_;
  if(meshing_engine_ != NULL)
    delete meshing_engine_;
  if(mesh_ != NULL)
    delete mesh_; 
}

Mesh* MainEngine::UpdateMesh() {
  if(mesh_ != NULL)
    meshing_engine_->MeshScene(mesh_, scene_);
  return mesh_;
}

void MainEngine::SaveSceneToMesh(const char* ojb_file_name) {
  if(mesh_ != NULL)
    return;
  meshing_engine_->MeshScene(mesh_, scene_);
  mesh_->WriteStl(object_file_name);
}

void MainEngine::ProcessFrame(Uchar4Image* rgb_image, 
                              ShortImage* raw_depth_image,
                              ImuMeasurement* imu_measurement) {
  if(imu_measurement == NULL)
    view_builder_->UpdateView(&view_, rgb_image, raw_depth_image, 
                              settings_->use_bilateral_filter_,
                              settings_->model_sensor_noise_);
  else 
    view_builder_->UpdateView(&view, rgb_image, raw_depth_image, 
                              settings_->use_bilateral_filter_, 
                              imu_measurement);
  if(!main_processing_active_)
    return;
  tracking_controller_->Track(tracking_state_, view_);
  if(fusion_active_)
    dense_mapper_->ProcessFrame(view_, tracking_state_, scene_, 
                                render_state_live_);
  tracking_controller_->Prepare(tracking_state_, view_, render_state_live_);
}

Vector2i MainEngine::GetImageSize() const {
  return render_state_live_->raycast_image_->num_dims_;
}

void MainEngine::GetImage(Uchar4Image* out, ImageType image_type, 
                          Pose* pose, Intrinsics* intrinsics) {
  if(view_ == NULL)
     return;
  out->Clear();
  switch(image_type) {
    case MainEngine::IMAGE_ORIGIN_RGB:
      out->ChangeDimension(view_->rgb_->num_dims_);
      if(settings_->device_type_ == Settings::DEVICE_GPU)
        out->SetFrom(view_->rgb_, utils::MemoryBlock<Vector4u>::GPU_TO_CPU);
      else
        out->SetFrom(view_->rgb_, utils::MemoryBlock<Vector4u>::CPU_TO_GPU);
      break;
    case MainEngine::IMAGE_ORIGIN_DEPTH:
      out->ChangeDimension(view_->depth_->num_dims_);
      if(settings_->tracker_type_ ==
         semantic_mapping::objects::Settings::TRACKER_WICP) {
         if(settings_->device_type_ == Settings::DEVICE_GPU)
           view_->depth_uncertainty_->UpdateHostFromDevice();
         VisualisationEngine<Voxel, VoxelIndex>::WeightToUchar4(out, 
                                                 view_->depth_uncertainty_);
      } else {
        if(settings_->device_type_ == Settings::DEVICE_GPU)
          view_->depth_->UpdateHostFromDevice();
        VisualisationEngine<Voxel, VoxelIndex>::DepthToUchar4(out, 
                                                view_->depth_);
      }
      break;
    case MainEngine::IMAGE_SCENERAYCAST:
      utils::Image<Vector4u>* src_image = render_state_live_->raycast_image_;
      out->ChangeDimension(src_image->num_dims_);
      if(settings_->device_type_ == Settings::DEVICE_GPU)
        out->SetFrom(src_image, utils::MemoryBlock<Vector4u>::GPU_TO_CPU);
      else 
        out->SetFrom(src_image, utils::MemoryBlock<Vector4u>::CPU_TO_GPU);
      break;
    case MainEngine::IMAGE_FREE_CAMERA_SHADED:
    case MainEngine::IMAGE_FREE_CAMERA_COLOR_FROM_VOLUME:
    case MainEngine::IMAGE_FREE_CAMERA_COLOR_FROM_NORMAL:
    {
      VisualisationEngine::RenderImageType type 
                           = VisualisationEngine::RENDER_SHADED_GREYSCALE;  
      if(image_type == MainEngine::IMAGE_FREE_CAMERA_COLOR_FROM_VOLUME) 
        type = VisualisationEngine::RENDER_COLOR_FROM_VOLUME;
      else if(image_type == MainEngine::IMAGE_FREE_CAMERA_COLOR_FROM_NORMAL)
        type = VisualisationEngine::RENDER_COLOR_FROM_NORMAL;
      if(render_state_free_view_ == NULL)
        render_state_free_view_ = visualisation_engine_->CreateRenderState(
                                                         out->num_dims_);
      visualisation_engine_->FindVisibleBlocks(pose, intrinsics, 
                                               render_state_free_view_);
      visualisation_engine_->CreateExpectedDepths(pose, intrinsics,
                                                  render_state_free_view_);
      visualisation_engine_->RenderImage(pose, intrinsics, 
                             render_state_free_view_, 
                             render_state_free_view_->raycast_image_,
                             type);
      if(settings_->device_type_ == Settings::DEVICE_GPU) 
        out->SetFrom(render_state_free_view_->raycast_image_, 
                     utils::MemoryBlock<Vector4u>::GPU_TO_CPU);
      else 
        out->SetFrom(render_state_free_view_->raycast_image_,
                     utils::MemoryBlock<Vector4u>::CPU_TO_GPU);
      break;
    }
    case MainEngine::IMAGE_UNKNOWN:
      break;  
  } 
}

void MainEngine::TurnOnIntegration() {
  fusion_active_ = true;
}

void MainEngine::TurnOffIntegration() {
  fusion_active_ = false;
}

void MainEngine::TurnOnMainProcessing() {
  main_processing_active_ = true;
}

void MainEngine::TurnOffMainProcessing() {
  main_processing_active_ = false;
} 
