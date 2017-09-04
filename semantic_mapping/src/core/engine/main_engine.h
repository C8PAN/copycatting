#ifndef MAIN_ENGINE_H
#define MAIN_ENGINE_H

namespace semantic_mapping {
namespace main_engine {

class MainEngine {
public:

private:
  const Settings* settings_;
  bool fusion_active_, main_processing_active_;
  LowLevelEngine* low_level_engine_; 
  VisualisationEngine* visualisation_engine_;
  MeshingEngine<Voxel, VoxelIndex>* meshing_engine_;
  Mesh* mesh_;
  ViewBuilder* view_builder_;
  DenseMapper<Voxel, VoxelIndex>* dense_mapper_;
  TrackingController* tracking_controller_;
  Tracker* tracker_;
  ImuCalibrator* imu_calibrator_;
};
		
} /* main_engine */ 		
} /* semantic_mapping */ 


#endif /* end of include guard: MAIN_ENGINE_H */
