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
  VoxelBlockHash(MemoryDeviceType memory_type) {
    this->memory_type_ = memory_type;
    hash_entries_ = new utils::MemeoryBlock<HashEntry>(total_entries_num, 
                                                       memory_type);
    excess_allocation_list_ = new utils::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE,
                                                          memory_type);
  }   

  ~VoxelBlockHash() {
    delete hash_entries_;
    delete excess_allocation_list_;
  }  

  const HashEntry* GetEntries() const { 
    return hash_entries_->GetData(memory_type_);
  }

  HashEntry* GetEntries() {
    return hash_entries_->GetData(memory_type_);
  }

  const IndexData* GetIndexData() const {
    return hash_entries_->GetData(memory_type_);
  }

  IndexData* GetIndexData() {
    return hash_entries_->GetData(memory_type_);
  }

  const int* GetExcessAllocationList() const {
    return excess_allocation_list_->GetData(memory_type_);
  }

  int* GetExcessAllocationList() {
    return excess_allocation_list_->GetData(memory_type_);
  }

  int GetLastFreeExcessListId() { return last_free_excess_list_id_; } 

  void SetLastFreeExcessListId(int last_free_excess_list_id) {
    this->last_free_excess_list_id_ = last_free_excess_list_id;
  }

  int GetAllocatedVoxelBlocksNum() { return SDF_LOCAL_BLOCK_NUM; }
  int GetVoxelBlockSize() { return SDF_BLOCK_SIZE3; }

  VoxelBlockHash(const VoxelBlockHash&);
  VoxelBlockHash& operator=(const VoxelBlockHash&);
};

}
}

#endif
