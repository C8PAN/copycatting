#ifndef CALIBRATION_IO_H
#define CALIBRATION_IO_H

#include <iostream>

namespace semantic_mapping {
namespace objects {
	
bool ReadIntrinsics(std::istream& src, Intrinsics& dest);
bool ReadIntrinsics(const char* file_name, Intrinsics& dest);
bool ReadExtrinsics(std::istream& src, Extrinsics& dest);
bool ReadExtrinsics(const char* file_name, Extrinsics& dest);
bool ReadDisparityCalibration(std::istream& src, DisparityCalibration& dest);
bool ReadDisparityCalibration(const char* file_name, DisparityCalibration& dest);
bool ReadRGBDCalibration(std::istream& src, RGBDCalibration& dest);
bool ReadRGBDCalibration(const char* file_name, RGBDCalibration& dest);

bool ReadRGBDCalibration(const char* rgb_intrinsics_file, 
                         const char* depth_intrinsics_file,
                         const char* disparity_calibration_file,
                         const char* extrinsics_file,
                         RGBDCalibration& dest);
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: CALIBRATION_IO_H */
