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

private:
  /* data */
};
	
} /* engine */ 	
} /* semantic_mapping */ 


#endif /* end of include guard: VISUALISATION_ENGINE_H */
