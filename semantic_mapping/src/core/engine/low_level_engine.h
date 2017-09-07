#ifndef LOW_LEVEL_ENGINE_H
#define LOW_LEVEL_ENGINE_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
	
class LowLevelEngine {
public:
  virtual void CopyImage(Uchar4Image* image_out, 
                         const Uchar4Image* image_in) const = 0;
  virtual void CopyImage(FloatImage* image_out, 
                         const FloatImage* image_in) const = 0;
  virtual void CopyImage(Float4Image* image_out, 
                         const Float4Image* image_in) const = 0;

  virtual void FilterSubsample(Uchar4Image* image_out, 
                               const Uchar4Image* image_in) const = 0;
  virtual void FilterSubsampleWithHoles(FloatImage* image_out, 
                               const FloatImage* image_in) const = 0;
  virtual void FilterSubsampleWithHoles(Float4Image* image_out,
                               const Float4Image* image_in) const = 0;
  
  virtual void GradientX(Short4Image* gradient_out, 
                         const Uchar4Image* image_in) const = 0;
  virtual void GradientY(Short4Image* gradient_out,
                         const Uchar4Image* image_in) const = 0;

  LowLevelEngine() {}
  virtual ~LowLevelEngine() {}
};
	
} /* engine */ 		
} /* semantic_mapping */ 


#endif /* end of include guard: LOW_LEVEL_ENGINE_H */
