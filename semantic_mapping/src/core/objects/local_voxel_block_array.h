#ifndef LOCAL_VOXEL_BLOCK_ARRAY_H
#define LOCAL_VOXEL_BLOCK_ARRAY_H

#include "defines.h"

#include <stdlib.h>

namespace semantic_mapping {
namespace objects {

template<class TVoxel>
class LocalVoxelBlockArray {
public:
  inline TVoxel* GetVoxelBlocks() {
    return voxel_blocks_->GetData(memory_type_);
  }

  inline const TVoxel* GetVoxelBlocks() const {
    return voxel_blocks_->GetData(memory_type_);
  }

  int* GetAllocationList() {
    return allocation_list_->GetData(memory_type_);
  }

  int last_free_block_id_;
  int allocated_size_;

  LocalVoxelBlockArray(MemoryDeviceType memory_type, int num_blocks, 
                       int block_size) {
    this->memory_type_ = memory_type;
    allocated_size_ = num_blocks*block_size;
    voxel_blocks_ = new utils::MemoryBlock<TVoxel>(allocated_size_, memory_type);
    allocation_list_ = new utils::MemoryBlock<int>(num_blocks, memory_type);
  }

  ~LocalVoxelBlockArray() {
    delete voxel_blocks_;
    delete allocation_list_;
  }

  LocalVoxelBlockArray(const LocalVoxelBlockArray&);
  LocalVoxelBlockArray& operator=(const LocalVoxelBlockArray&);

private:
  utils::MemoryBlock<TVoxel>* voxel_blocks_;
  utils::MemoryBlock<int>* allocation_list_;
  MemoryDeviceType memory_type_;
};
		
} /* objects */ 
} /* semantic_mapping */ 



#endif /* end of include guard: LOCAL_VOXEL_BLOCK_ARRAY_H */
