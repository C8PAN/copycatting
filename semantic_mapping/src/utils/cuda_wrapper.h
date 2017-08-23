#ifndef CUDA_WRAPPER_H
#define CUDA_WRAPPER_H

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>

#ifndef CudaSafeCall
#define CudaSafeCall(error) utils::__cudaSafeCall(error, __FILE__, __LINE__)

namespace utils {

inline void __cudaSafeCall(cudaError error, const char *file, const int line) {
  if(cudaSuccess != err) {
    printf("%s(%i): cudaSafeCall() Runtime API error: %s \n",
           file, line, cudaGetErrorString(error));
    exit(-1);
  }
}

}

#endif

#endif
