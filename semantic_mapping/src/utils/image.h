#ifndef IMAGE_H_Q8EXQTG4
#define IMAGE_H_Q8EXQTG4

#include "memory_block.h"
#include "vector.h"

namespace utils {

template<typename T>
class Image : public MemoryBlock<T> {
public:
  Vector<2> num_dims_;
  
  Image(Vector2<int> num_dims, bool allocate_cpu, bool allocate_gpu) 
       : MemoryBlock<T>(num_dims.x*num_dims.y, allocate_cpu, allocate_gpu) {
    this->num_dims_ = num_dims;
  }

  Image(bool allocate_cpu, bool allocate_gpu) 
       : MemoryBlock<T>(0, allocate_cpu, allocate_gpu) {
    this->num_dims_ = Vector2<int>(0,0);
  }

  Image(Vector2<int> num_dims, MemoryDeviceType memory_type) 
       : MemoryBlock<T>(num_dims.x*num_dims.y, memory_type) {
    this->num_dims_ = num_dims;
  }

  void ChangeDimension(Vector2<int> new_dims) {
    if(new_dims != num_dims_) {
      this->num_dims_ = new_dims;
      bool allocate_cpu = this->is_allocated_cpu_;
      bool allocate_gpu = this=>is_allocated_gpu_;
      this->Free();
      this->Allocate(new_dims.x*new_dims.y, allocate_cpu, allocate_gpu);  
    }
  } 

  Image(const Image&);
  Image& operator=(const Image&); 
};
	
} /* utils */ 


#endif /* end of include guard: IMAGE_H_Q8EXQTG4 */
