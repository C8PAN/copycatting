#ifndef VISUALISATION_ENGINE_H
#define VISUALISATION_ENGINE_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
	
class VisualisationEngine {
public:
  enum RenderImageType { 
    RENDER_SHADED_GREYSCALE, RENDER_COLOR_FROM_VOLUME, RENDER_COLOR_FROM_NORMAL
  };

  virtual ~VisualisationEngine() {}

  static void DepthToUchar4(Uchar4Image* dst, FloatImage* src);
  static void NormalToUchar4(Uchar4Image* dst, Float4Image* src);
  static void WeightToUchar4(Uchar4Image* dst, FloatImage* src);

  virtual void FindVisibleBlocks(const Pose* pose, Intrinsics* intrinsics,
                                 RenderState* render_state) const = 0; 
 
  virtual void CreateExpectedDepths(const Pose* pose, 
          const Intrinsics* intrinsics, RenderState* render_state) const = 0;

  virtual void RenderImage(const Pose* pose, const Intrinsics* intrinsics,
          const RenderState* render_state, Uchar4Image* output_image,
          RenderImageType type = RENDER_SHADED_GREYSCALE) const = 0;

  virtual void FindSurface(const Pose* pose, const Intrinsics* intrinsics,
          const RenderState* render_state) const = 0;

  virtual void CreatePointCloud(const View* view, TrackingState* tracking_state,
          RenderState* render_state, bool skip_points) const = 0;

  virtual void CreateIcpMap(const View* view, TrackingState* tracking_state,
          RenderState* render_state) const = 0;

  virtual void ForwardRender(const View* view, TrackingState* tracking_state,
          RenderState* render_state) const = 0;
  
  virtual RenderImage* CreateRenderState(const Vector2i& image_size) const = 0;
};

template<class TIndex>
struct IndexToRenderState {
  typedef RenderState type;
}; 

template<>
struct IndexToRenderState<VoxelBlockHash> {
  typedef RenderStateVoxelHash type;
}

template<class TVoxel, class TIndex>
class VisualisationEngine : public VisualisationEngine {
public:
  const Scene<TVoxel, TIndex>* scene_;
 
  VisualisationEngine(const Scene<TVoxel, TIndex>* scene) {
    this->scene_ = scene;
  }
 
protected:
  virtual typename IndexToRenderState<TIndex>::type* CreateRenderState(
                   const Vector2i& image_size) const = 0;
};

	
} /* engine */ 	
} /* semantic_mapping */ 


#endif /* end of include guard: VISUALISATION_ENGINE_H */
