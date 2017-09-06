#ifndef COMPOSITE_TRACKER_H
#define COMPOSITE_TRACKER_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
		
class CompositeTracker {
public:
  SetTracker(Tracker* tracker, int tracker_id) {
    if(trackers_[tracker_id] != NULL) 
      delete trackers_[tracker_id];
    trackers_[tracker_id] = tracker;
  }

  CompositeTracker(int num_trackers) {
    trackers_ = new Tracker*[num_trackers];
    for(int i=0; i<num_trackers; i++)
      trackers_[i] = NULL;
    this->num_trackers_ = num_trackers;
  }

  ~CompositeTracker() {
    for(int i=0; i<num_trackers_; i++) 
      if(trackers_[i] != NULL)
        delete trackers_[i];
    delete[] trackers_;
  }

  void TrackCamera(TrackingState* tracking_state, const View* view) {
    for(int i=0; i<num_trackers_; i++)
      trackers_[i]->TrackCamera(tracking_state, view);
  }

  void UpdateInitialPose(TrackingState* tracking_state) {
    for(int i=0; i<num_trackers_; i++)
      trackers_[i]->UpdateInitialPose(tracking_state);
  }

  CompositeTracker(const CompositeTracker&);
  CompositeTracker& operator=(const CompositeTracker&);

private:
  Tracker** trackers_;
  int num_trackers_;
};

} /* engine */ 	
} /* semantic_mapping */ 



#endif /* end of include guard: COMPOSITE_TRACKER_H */
