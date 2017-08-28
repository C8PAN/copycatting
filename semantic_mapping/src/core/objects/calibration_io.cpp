#include "calibration_io.h"

#include <fstream>
#include <sstream>

using namespace semantic_mapping::objects;

bool semantic_mapping::objects::ReadIntrinsics(std::istream& src, 
                                               Intrinsics& dest) {
  float size_x, size_y;
  float focal_length[2], center_point[2];
  src >> size_x >> size_y;
  src >> focal_length[0] >> focal_length[1];
  src >> center_point[0] >> center_point[1];
  
  if(src.fail())
    return false;
  
  dest.SetFrom(focal_length[0], focal_length[1], center_point[0], 
               center_point[1], size_x, size_y);
  return true;
}

bool semantic_mapping::objects::ReadIntrinsics(const char* file_name, 
                                               Intrinsics& dest) {
  std::ifstream f(file_name);
  return semantic_mapping::objects::ReadIntrinsics(f, dest);
}

bool semantic_mapping::objects::ReadExtrinsics(std::istream& src, 
                                               Extrinsics& dest) {
  Matrix4f calib;
  src >> calib.m00 >> calib.m10 >> calib.m20 >> calib.m30;
  src >> calib.m01 >> calib.m11 >> calib.m21 >> calib.m31;
  src >> calib.m02 >> calib.m12 >> calib.m22 >> calib.m32;
  calib.m03 = 0.0f; 
  calib.m13 = 0.0f; 
  calib.m23 = 0.0f;
  calib.m33 = 1.0f;

  if(src.fail())
    return false;
  
  dest.SetFrom(calib);
  return true;
}

bool semantic_mapping::objects::ReadExtrinsics(const char* file_name,
                                               Extrinsics& dest) {
  std::ifstream f(file_name);
  return semantic_mapping::objects::ReadExtrinsics(f, dest);
}

bool semantic_mapping::objects::ReadDisparityCalibration(std::istream& src,
                                DisparityCalibration& dest) {
  std::string word;
  src >> word;
  if(src.fail())
    return false;
  DisparityCalibration::TransferType type 
    = DisparityCalibration::TRANSFER_KINECT;
  float a, b;
  if(word.compare("kinect")==0) {
    type = DisparityCalibration::TRANSFER_KINECT;
    src >> a;
  } else if(word.compare("affine")==0) {
    type = DisparityCalibration::TRANSFER_AFFINE; 
    src >> a;
  } else {
    std::stringstream word_stream(word);
    word_stream >> a;
    if(word_stream.fail())
      return false;
  }

  src >> b;
  if(src.fail())
    return false;
  
  if((a==0.0f) && (b==0.0f)) {
    type == DisparityCalibration::TRANSFER_AFFINE;
    a = 1.0f/1000.0f;
    b = 0.0f;
  } 

  dest.SetFrom(a, b, type);
  return true;
}

bool semantic_mapping::objects::ReadDisparityCalibration(const char* file_name,
                                DisparityCalibration& dest) {
  std::ifstream f(file_name);
  semantic_mapping::objects::ReadDisparityCalibration(f, dest);
}

bool semantic_mapping::objects::ReadRGBDCalibration(std::istream& src,
                                RGBDCalibration& dest) {
  if(!semantic_mapping::objects::ReadIntrinsics(src, dest.intrinsics_rgb_))
    return false;
  if(!semantic_mapping::objects::ReadIntrinsics(src, dest.intrinsics_d_))
    return false;
  if(!semantic_mapping::objects::ReadExtrinsics(src, 
                                 dest.transfer_rgb_to_depth_))
    return false;
  if(!semantic_mapping::objects::ReadDisparityCalibration(src, 
                                 dest.disparity_calibration_))
    return false;
  
  return true;
}

bool semantic_mapping::objects::ReadRGBDCalibration(const char* file_name,
                                                    RGBDCalibration& dest) {
  std::ifstream f(file_name);
  return semantic_mapping::objects::ReadRGBDCalibration(f, dest);
}

bool semantic_mapping::objects::ReadRGBDCalibration(
                                const char* rgb_intrinsics_file,
                                const char* depth_intrinsics_file,
                                const char* disparity_calib_file,
                                const char* extrinsics_file,
                                RGBDCalibration& dest) {
  bool result = true;
  result &= semantic_mapping::objects::ReadIntrinsics(rgb_intrinsics_file, 
                                       dest.intrinsics_rgb_);
  result &= semantic_mapping::objects::ReadIntrinsics(depth_intrinsics_file,
                                       dest.intrinsics_d_);
  result &= semantic_mapping::objects::ReadExtrinsics(extrinsics_file, 
                                       dest.transfer_rgb_to_depth_);
  result &= semantic_mapping::objects::ReadDisparityCalibration(
                                       disparity_calib_file,
                                       dest.disparity_calibration_);
  return result;
}
