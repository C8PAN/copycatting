#ifndef VOXEL_BLOCK_HASH_H
#define VOXEL_BLOCK_HASH_H

#include <stdlib.h>

namespace semantic_mapping {
namespace objects {

class VoxelBlockHash {
public:
  typedef HashEntry IndexData;
  struct IndexCache {
    Vector3i block_pose;
    int block_ptr;
    _CPU_AND_GPU_CODE_ IndexCache() : block_pose(0x7fffffff), block_ptr(-1) {}
  };  

  static const int total_entries_num = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
  static const int voxel_blick_size 
                   = SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;

private:
  int last_free_excess_list_id_;
  utils::MemoryBlock<HashEntry> *hash_entries_;
  utils::MemoryBlock<int> *excess_allocation_list_;
  MemoryDeviceType memory_type_;

public:
    
};

}
}

#endif
