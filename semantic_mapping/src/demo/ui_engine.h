#ifndef UI_ENGINE_H
#define UI_ENGINE_H

#include <vector>

namespace semantic_mapping {
namespace demo {
	
class UiEngine {
public:
  static UiEngine* Instance() {
    if(instance_==NULL)
      instance_ = new UiEngine();
    return instance_;
  }

  static void GlutDisplayFunction();
  static void GlutIdleFunction();
  static void GlutKeyUpFunction(unsigned char key, int x, int y);
  static void GlutMouseButtonFunction(int button, int state, int x, int y);
  static void GlutMouseMoveFunction(int x, int y);
  static void GlutMouseWheelFunction(int button, int dir, int x, int y);

  const Vector2i& GetWindowSize() const {
    return win_size_;
  }

  float processed_time_;
  int processed_frame_num_;
  char* out_folder_;
  bool need_refresh_;
  Uchar4Image* save_image_;

  void Initialise(int& argc, char** argv, ImageSourceEngine* image_source,
                 MainEngine* main_engine, const char* out_folder, 
                 Settings::DeviceType device_type);

  void ShutDown();
  void Run();
  void ProcessFrame();
  void GetScreenshot(Uchar4Image* dest) const; 
  void SaveScreenshot(const char* file_name) const;
  void SaveSceneToMesh(const char* file_name) const;

private:
  static UiEngine* instance_;
  
  enum MainLoopAction {
    PROCESS_PAUSED, PROCESS_FRAME, PROCESS_VIDEO, EXIT, SAVE_TO_DISK 
  }main_loop_action_;
  
  struct UiColorMode {
    const char* name;
    MainEngine::ImageType type;
    UiColorMode(const char* _name, MainEngine::ImageType _type) :
      name(_name), type(_type) {}
  };

  std::vector<UiColorMode> color_modes_;
  int current_color_mode_;
  Settings internal_settings_;
  ImageSourceEngine* image_source_;
  MainEngine* main_engine_;
  StopWatchInterface* timer_instance_;
  StopWatchInterface* timer_average_;

  static const int NUM_WIN = 3;
  Vector4f win_reg_[NUM_WIN];
  Vector2i win_size_;
  uint texture_id_[NUM_WIN];
  Uchar4Image* out_image_[NUM_WIN];
  MainEngine::ImageType out_image_type_[NUM_WIN];
  Uchar4Image* input_imu_measurement_;
  bool free_view_active_;
  bool intergration_active_;
  Pose free_view_pose_;
  Intrinsics free_view_intrinsics_;
  int mouse_state_;
  Vector2i mouse_last_click_;
  int current_frame_num_;
  bool is_recording_; 
};
	
} /* demo */ 
} /* semantic_mapping */ 


#endif /* end of include guard: UI_ENGINE_H */
