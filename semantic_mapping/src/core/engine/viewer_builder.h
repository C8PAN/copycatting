#ifndef VIEWER_BUILDER_H
#define VIEWER_BUILDER_H

using namespace semantic_mapping::objects;

namespace semantic_mapping {
namespace engine {
	
class ViewerBuilder {
public:
  virtual void ConvertDisparityToDepth(FloatImage* depth_out, 
                                       const ShortImage* disparity_in, 
                                       const Intrinsics* depth_intrinsics,
                                       Vector2f disparity_calib_params) = 0;

  virtual void ConvertDepthAffineToFloat(FloatImage* depth_out, 
                                         const ShortImage* depth_in, 
                                         Vector2f depth_calib_params) = 0;

  virtual void DepthFiltering(FloatImage* image_out, 
                              const FloatImage* image_in) = 0; 

  virtual void ComputeNormalAndWeights(Float4Image* normal_out, 
                                       FloatImage* sigma_z_out,
                                       const FloatImage* depth_in,
                                       Vector4f intrinsics) = 0;

  virtual void UpdateView(View** view, Uchar4Image* rgb_image, 
                          ShortImage* raw_depth_image, bool use_bilateral_filer,
                          bool model_sensor_noise=false) = 0;
  
  virtual void UpdateView(View** view, Uchar4Image* rgb_image, 
                          FloatImage* depth_image) = 0;

  virtual void UpdateView(View** view, Uchar4Image* rgb_image, 
                          ShortImage* depth_image, bool use_bilateral_filer,
                          ImuMeasurement* imu_measurement) = 0;
 
  ViewerBuilder(const RgbdCalibration* calib) {
    this->rgbd_calibration_ = calib;
    this->short_image_ = NULL;
    this->float_image_ = NULL:
  }

  virtual ~ViewerBuilder() {
    if(this->short_image_ != NULL)
      delete this->short_image_;
    if(this->float_image_ != NULL)
      delete this->float_image_;
  }

protected:
  const RgbdCalibration* rgbd_calibration_;
  ShortImage* short_image_;
  FloatImage* float_image_;
};	

} /* engine */ 	
} /* semantic_mapping */ 



#endif /* end of include guard: VIEWER_BUILDER_H */
