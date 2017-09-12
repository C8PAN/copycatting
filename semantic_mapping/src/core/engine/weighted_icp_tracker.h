#ifndef WEIGHTED_ICP_TRACKER_H
#define WEIGHTED_ICP_TRACKER_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
	
class WeightedIcpTracker : public Tracker {
public:
  void TrackCamera(TrackingState* tracking_state, const View* view);
  WeightedIcpTracker(Vector2i image_size, TrackerIterationType* tracking_regime,
                     int num_hierarchy_levels, int num_icp_run_till_level,
                     float distance_th, float termination_th, 
                     const LowLevelEngine* low_level_engine, 
                     MemoryDeviceType memory_type);
  virtual ~WeightedIcpTracker();
 
protected:
  float* distance_threshold_;
  int level_id_;
  TrackerIterationType iteration_type_;
  Matrix4f scene_pose_;
  SceneHierarchyLevel* scene_hierarchy_level_;
  TemplatedHierarchyLevel<FloatImage>* view_hierarchy_level_;
  TemplatedHierarchyLevel<FloatImage>* weight_hierarchy_level_;

  virtual int ComputeGrandH(float& f, float* nabla, float* hessian,
                            Matrix4f approx_inv_pose) = 0;

private:
  const LowLevelEngine* low_level_engine_;
  ImageHierarchy<SceneHierarchyLevel>* scene_hierarchy_;
  ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >* view_hierarchy_;
  ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >* weight_hierarchy_;
  TrackingState* tracking_state_;
  const View* view_;
  int num_iterations_per_level_;
  int num_icp_level_;
  float termination_threshold_;
  float hessian_[36];
  float nabla_[6];
  float step_[6];
  
  void PrepareForEvaluation();
  void SetEvaluationParameters(int level);
  void ComputeDelta(float* delta, float* nabla, float* hessian,
                    bool short_iteration) const;
  void ApplyDelta(const Matrix4f* old_pose, const float* delta, 
                  Matrix4f& new_pose) const;
  void SetEvaluationData(TrackingState* tracking_state, const View* view); 
};
	
} /* engine */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: WEIGHTED_ICP_TRACKER_H */
