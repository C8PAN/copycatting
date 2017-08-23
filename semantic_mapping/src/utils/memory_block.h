#ifndef MEMORY_BLOCK_H
#define MEMORY_BLOCK_H

#include <stdlib.h>
#include <string.h>

#ifndef MEMORY_DEVICE_TYPE
#define MEMORY_DEVICE_TYPE
enum MemoryDeviceType { MEMORY_DEVICE_CPU, MEMORY_DEVICE_GPU };
#endif

namespace utils {

template<typename T>
class MemoryBlock {
public:
  enum MemoryCopyDirection { CPU_TO_CPU, CPU_TO_GPU, GPU_TO_GPU, GPU_TO_CPU};
  // total number of allocated entries in the data array
  size_t data_size_;
  // get data pointer on cpu or gpu
  inline T* GetData(MemoryDeviceType memory_type) {
    switch(memory_type) {
      case MEMORY_DEVICE_CPU: return data_cpu_;
      case MEMOERY_DEVICE_GPU: return data_gpu_;
    } 
    return 0;
  }

  MemoryBlock(size_t data_size, bool allocate_cpu, bool allocate_gpu) {
    this->is_allocated_cpu_ = false;
    this->is_allocated_gpu_ = false;
    Allocate(data_size, allocate_cpu, allocate_gpu);
    Clear();
  }

  MemoryBlock(size_t data_size, MemoryDeviceType memory_type) {
    this->is_allocated_cpu_ = false;
    this->is_allocated_gpu_ = false;
    switch(memory_type) {
      case MEMORY_DEVICE_CPU: Allocate(data_size, true, false); break;
      case MEMORY_DEVICE_GPU: Allocate(data_size, false, true); break; 
    }
    Clear();
  }

  void Clear(unsigned char dafault_value=0) {
    if(is_allocated_cpu_) 
      memset(data_cpu_, default_value, data_size_*sizeof(T));
    if(is_allocated_gpu_)
      CudaSafeCall(cudaMemset(data_ppu_, default_value, data_size_*sizeof(T)));
  }

  void UpdateDeviceFromHost() const {
    if(is_allocated_cpu_ && is_allocated_gpu_)
      CudaSafeCall(cudaMemcpy(data_gpu_, data_cpu_, data_size_*sizeof(T), 
                   cudaMemcpyHostToDevice));
  }

  void UpdateHostFromDevice() const {
    if(is_allocated_gpu_ && is_allocated_cpu) 
      CudaSafeCall(cudaMemcpy(data_cpu_, data_gpu_, data_size_*sizeof(T),
                   cudaMemcpyDeviceToHost));
  }

  void SetFrom(const MemoryBlock<t> *source, 
               MemoryCopyDirection memory_copy_direction) {
    switch(memory_copy_direction) {
      case CPU_TO_CPU:
        memcpy(this->data_cpu_, source->data_cpu_, source->data_size_*sizeof(T));
        break;
      case CPU_TO_GPU:
        CudaSafeCall(cudaMemcpyAsync(this->data_gpu_, source->data_cpu_,
                     source->data_size_*sizeof(T), cudaMemcpyHostToDevice));
        break;
      case GPU_TO_CPU:
        CudaSafeCall(cudaMemcpy(this->data_cpu_, source->data_gpu_, 
                     source->data_size_*sizeof(T), cudaMemcpyDeviceToHost));   
        break;
      case GPU_TO_GPU:
        CudaSafeCall(cudaMemcpyAsync(this->data_gpu_, source->data_gpu_, 
                     source->data_size_*sizeof(T), cudaMemcpyDeviceToDevice));
        break;
      default:
        break; 
    }
  }

  virtual ~MemoryBlock() { this->Free(); }

  void Allocate(size_t data_size, bool allocate_cpu, bool allocate_gpu) {
    Free();
    this->data_size_ = data_size;
    if(allocate_cpu) {
      int allocate_type = 0;
      if(allocate_gpu) 
        allocate_type = 1;
      switch(allocate_type) {
        case 0:
          if(data_size_==0) data_cpu_ = NULL;
          else data_cpu_ = new T[data_size_];
          break;
        case 1:
          if(data_size_==0) data_cpu_ = NULL;
          else CudaSafeCall(cudaMallocHost((void**)&data_cpu_, 
                            data_size_*sizeof(T)));
          break;
        default:
          break;
      }
      this->is_allocated_cpu_ = allocate_cpu;
    }
    if(allocate_gpu) {
      if(data_size_==0) data_gpu_ = NULL:
      else CudaSafeCall(cudaMalloc((void**)&data_gpu_, data_size_*sizeof(T)));
      this->is_allocated_gpu_ = allocate_gpu;
    }
  }

  void Free() {
    if(is_allocated_cpu_) {
      int allocate_type = 0;
      if(is_allocated_gpu_)
        allocate_type = 1;
      switch(allocate_type) {
        case 0:
          if(data_cpu_!=NULL) delete[] data_cpu_;
          break;
        case 1:
          if(data_cpu_1=NULL) CudaSafeCall(cudaFreeHost(data_cpu_));
          break;
        default:
          break;
      }
      is_allocated_cpu_ = false;
    } 
    if(is_allocated_gpu_) {
      if(data_gpu_!=NULL)
        CudaSafeCall(cudaFree(data_gpu_));
      is_allocated_gpu_ = false;
    }
  }
 
  // supress the default copy constructor and assignment operator
  MemoryBlock(const MemoryBlock&); 
  MemoryBlock& operator=(const MemoryBlock&) 

protected:
  bool is_allocated_cpu_, is_allocated_gpu_;
  T* data_cpu_;
  T* data_gpu_;  
};

}

#endif
