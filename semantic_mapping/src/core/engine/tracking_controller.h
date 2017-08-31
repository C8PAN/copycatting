#ifndef TRACKING_CONTROLLER
#define TRACKING_CONTROLLER

namespace semantic_mapping {
namespace engine {
	
class TrackingController {
public:
  void Track(TrackingState* tracking_state, const View* view);
 
  void Prepare(Trackingstate* tracking_state, const View* view, 
               RenderState* render_state);

  TrackingController(Tracker* tracker, 
                     const VisualisationEngine* visualisation_engine,
                     const LowLevelEngine* low_level_engine,
                     Settings* settings) {
    this->tracker_ = tracker;
    this->settings_ = settings;
    this->visualisation_engine_ = visualisation_engine;
    this->low_level_engine_ = low_level_engine;
    memory_type_ = (settings->device_type_ 
      == Settings::DEVICE_GPU ? MEMORY_DEVICE_GPU : MEMORY_DEVICE_CPU);
  }

  TrackingState* BuildTrackingState(const Vector2i& tracked_image_size) const {
    return new TrackingState(tracked_image_size, memory_type_);
  }

  static Vector2i GetTrackedImageSize(const Settings* settings, 
                                      const Vector2i& rgb_image_size,
                                      const Vector2i& depth_image_size) {
    return (settings->tracker_type_ 
      == Settings::TRACKER_COLOR ? rgb_image_size : depth_image_size);
  }

  TrackingController(const TrackingController&);
  TrackingController& operator=(const TrackingController&);

private:
  const Settings* settings_;
  VisualisationEngine* visualisation_engine_;
  LowLevelEngine* low_level_engine_;
  Tracker* tracker_;
  MemoryDviceType memory_type_;
};
	
} /* engine */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: TRACKING_CONTROLLER */
