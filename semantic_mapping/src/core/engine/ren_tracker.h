#ifndef REN_TRACKER_H
#define REN_TRACKER_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
	
template<class TVoxel, class TIndex>
class RenTracker : public Tracker {
public:
  void ApplyDelta(const Pose& old_pose, const float* delta, 
                  Pose& new_pose) const = 0;

  void NumParameters() const { return 6; }

  void TrackCamera(TrackingState* tracking_state, const View* view);

  RenTracker(Vector2i image_size, TrackerIterationType tracking_regime, 
             int num_hierarchy_levels, const LowLevelEngine* low_level_engine,
             const Scene<TVoxel, TIndex>* scene, MemoryDeviceType memory_type);

  virtual RenTracker() ;

private:
  TrackingState* tracking_state_;
  const LowLevelEngine* low_level_engine_;
  FloatImage* image1_;
  FloatImage* image2_;
  const View* view_;
  int* num_iterations_per_level_;
 
  void PrepareForEvaluation(const View* view);

protected:
  const Scene<TVoxel, TIndex>* scene_;
  ImageHierarchy<TemplatedHierarchyLevel<Float4Image> >* view_hierarchy_;
  int level_id_;
  bool rotation_only_;
  float hessian_[36];
  float nabla_[6];
  
  virtual void FOneLevel(float* f, Matrix4f inv_mat) = 0;
  virtual void GOneLevel(float* gradient, float* hessian, 
                         Matrix4f inv_mat) const = 0;
  virtual void UnprojectDepthToCamera(FloatImage* depth, 
                                      Float4Image* up_point_cloud,
                                      const Vector4f& intrinsic) = 0;
};

	
} /* engine */ 	
} /* semantic_mapping */ 



#endif /* end of include guard: REN_TRACKER_H */
