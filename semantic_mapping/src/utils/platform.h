#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdexcept>

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define _CPU_AND_GPU_CODE_ __device__
#else
#define _CPU_AND_GPU_CODE_ 
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define _CPU_AND_GPU_CONSTANT_ __constant__
#else
#define _CPU_AND_GPU_CONSTANT_
#endif

#define DIEWITHEXCEPTION(x) throw std::runtime_error(x)

#endif /* end of include guard: PLATFORM_H */
