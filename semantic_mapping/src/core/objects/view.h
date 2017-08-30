#ifndef VIEW_H_J0YXKZIU
#define VIEW_H_J0YXKZIU

namespace semantic_mapping {
namespace objects {

class View {
public:
  RGBDCalibration* rgbd_calib_;
  Uchar4Image* rgb_;
  FloatImage* depth_;
  Float4Image* depth_normal_;
  FloatImage* depth_uncertainty_;

  View(const RGBDCalibration* rgbd_calib, Vector2i rgb_image_size, 
       Vector2i depth_image_size, bool use_gpu) {
    this->rgbd_calib_ = new RGBDCalibration(*rgbd_calib);
    this->rgb_ = new Uchar4Image(rgb_image_size, true, use_gpu);
    this->depth_ = new FloatImage(depth_image_size, true, use_gpu);
    this->depth_normal_ = NULL;
    this->depth_uncertainty_ = NULL;
  } 

  virtual ~View() {
    delete rgbd_calib_;
    delete rgb_;
    delete depth_;
    if(depth_normal_ != NULL)
      delete depth_normal_;
    if(depth_uncertainty_ != NULL)
      delete depth_uncertainty_;
  }

  View(const View&);
  View& operator=(const View&);
};
		
} /* objects */ 
} /* semantic_mapping */ 



#endif /* end of include guard: VIEW_H_J0YXKZIU */
