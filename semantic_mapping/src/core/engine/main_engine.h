#ifndef MAIN_ENGINE_H
#define MAIN_ENGINE_H

namespace semantic_mapping {
namespace main_engine {

class MainEngine {
public:
  enum ImageType { 
       IMAGE_ORIGIN_RGB, IMAGE_ORIGIN_DEPTH, IMAGE_SCENERAYCAST, 
       IMAGE_FREE_CAMERA_SHADED, IMAGE_FREE_CAMERA_COLOR_FROM_VOLUME,
       IMAGE_FREE_CAMERA_COLOR_FROM_NORMAL, IMAGE_UNKNOWN
  };

  View* GetView() { return view_;}
  
  TrackingState* GetTrackingState() { return tracking_state_; }

  Scene<Voxel, VoxelIndex>* GetScene() { return scene_; }

  void ProcessFrame(Uchar4Image* rgb_image, ShortImage* raw_depth_image,
                    ImuMeasurement* imu_measurement=NULL);

  Mesh* GetMesh() { return mesh; }
  
  Mesh* UpdateMesh();
  
  void SaveSceneToMesh(const char* object_file_name);

  Vector2i GetImageSize() const;

  void GetImage(Uchar4Image* out, ImageType image_type, Pose* pose=NULL, 
                Intrinsics* intrinsics=NULL);
 
  void TurnOnIntegration();
  void TurnOffIntegration();

  void TurnOnMainProcessing();
  void TurnOffMainProcessing();

  MainEngine(const Settings* settings, RgbdCalibration* calib,
             Vector2i image_size_rgb, Vector2i image_size_d=Vector2i(-1,-1));
  ~MainEngine();

private:
  const Settings* settings_;
  bool fusion_active_; 
  bool main_processing_active_;
  LowLevelEngine* low_level_engine_; 
  VisualisationEngine* visualisation_engine_;
  MeshingEngine<Voxel, VoxelIndex>* meshing_engine_;
  Mesh* mesh_;
  ViewBuilder* view_builder_;
  DenseMapper<Voxel, VoxelIndex>* dense_mapper_;
  TrackingController* tracking_controller_;
  Tracker* tracker_;
  ImuCalibrator* imu_calibrator_;
  View* view_;
  TrackingState* tracking_state_;
  Scene<Voxel, VoxelIndex>* scene_;
  RenderState* render_state_live_;
  RenderState* render_state_free_view_;
};
		
} /* main_engine */ 		
} /* semantic_mapping */ 


#endif /* end of include guard: MAIN_ENGINE_H */
