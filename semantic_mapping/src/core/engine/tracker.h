#ifndef TRACKER_H
#define TRACKER_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
	
class Tracker {
public:
  virtual void TrackCamera(TrackingState* tracking_state, const View* view) = 0;
  virtual void UpdateInitialPose(TrackingState* tracking_state) {}
  virtual ~Tracker() {}
};
	
} /* engine */ 	
} /* semantic_mapping */ 



#endif /* end of include guard: TRACKER_H */
