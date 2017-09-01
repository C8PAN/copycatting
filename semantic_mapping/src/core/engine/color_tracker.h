#ifndef COLOR_TRACKER_H
#define COLOR_TRACKER_H

using namespace semantic_mapping::objects;

namespace semantic_mappingt {
namespace engine {
	
class ColorTracker : public Tracker {
public:
  
  class EvaluationPoint {
  public:
    float GetF() { return cache_f_; }
    
    const float* GetNabla() {
      if(cache_nabla_ == NULL)
        ComputeGradients(false);
      return cache_nabla_;
    }

    const float* GetHessian() {
      if(cache_hessian_ == NULL)
        ComputeGradients(true);
      return cache_hessian_;
    }

    const Pose& GetParameter() const {
      return *pose_
    }

    EvaluationPoint(Pose* pose, ColorTracker* parent);
   
    ~EvaluationPoint() {
      delete pose_;
      if(cache_nabla_ != NULL)
        delete[] cache_nabla_;
      if(cache_hessian_ != NULL)
        delete[] cache_hessian_;
    }

  protected:
    void ComputeGradients(bool requires_hessian);
    Pose* pose_;
    const ColorTracker* parent_;
    float cache_f_;
    float* cache_nabla_;
    float* cache_hessian_;    
  };  

  EvaluationPoint* EvaluateAt(Pose* pose) const {
    return new EvaluationPoint(pose, this);
  }

  int NumParameters() const {
    return (iteration_type_ == TRACKER_ITERATION_ROTATION) ? 3 : 6;
  }

  virtual void FOneLevel(float* f, Pose* pose) = 0;

  virtual void GOneLevel(float* gradient, float* hessian, Pose* pose) const = 0;
  
  void ApplyDelta(const Pose& pose_old, const float* delta, 
                  Pose& pose_new) const; 
  
  void TrackCamera(TrackingState* tracking_state, const View* view);

  ColorTracker(Vector2i image_size, TrackerIterationType* tracking_regime,
               int num_hierarchy_levels, const LowLevelEngine* low_level_engine,
               MemoryDeviceType memory_type);

  virtual ~ColorTracker();

private:
  const LowLevelEngine* low_level_engine_;
  void ParpareForEvaluation(const View* view);

protected:
  TrackerIterationType iteration_type_;
  TrackingState* tracking_state_;
  const View* view_;
  ImageHierarchy<ViewHierarchyLevel>* view_hierarchy_;
  int level_id_;
  int counted_points_valid_; 
};
	
} /* engine */ 	
} /* semantic_mappingt */ 



#endif /* end of include guard: COLOR_TRACKER_H */
