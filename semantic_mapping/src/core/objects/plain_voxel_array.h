#ifndef PLAIN_VOXEL_ARRAY_H
#define PLAIN_VOXEL_ARRAY_H

#include <stdlib.h>

namespace semantic_mapping {
namespace objects {

class PlainVoxelArray {
public:
  struct VoxelArrayInfo {
    // size in voxels
    Vector3i size;
    // offset of the lower left front corner of the volume in voxels
    Vector3i offset;
    
    VoxelArrayInfo(void) {
      size.x = size.y = size.z = 512;
      offset.x = offset.y = -256;
      offset.z = 0;
    }
  };
 
  typedef VoxelArrayInfo IndexData;
  struct IndexCache {};

private:
  utils:MemoryBlock<IndexData> *index_data_;
  MemoryDeviceType memory_type_; 

public:
  PlainVoxelArray(MemoryDeviceType memory_type) {
    this->memory_type_ = memory_type;
    if(memory_type==MEMORY_DEVICE_GPU) 
      index_data_ = new utils::MemoryBlock<IndexData>(1, true, true);
    else 
      index_data_ = new utils::MemoryBlock<IndexData>(1, true, false);
    index_data_->GetData(MEMORY_DEVICE_CPU)[0] = IndeData();
    index_data_->UpdateDeviceFromHost();
  }

  ~PlainVoxelArray() { delete index_data_; }

  int GetAllocatedVoxelBlocksNum() { return 1; }
  
  int GetVoxelBlockSize() {
    return index_data_->GetData(MEMORY_DEVICE_CPU)->size.x*
           index_data_->GetData(MEMORY_DEVICE_CPU)->size.y*
           index_data_->GetData(MEMORY_DEVICE_CPU)->size.z;
  }

  const Vector3i GetVolumeSize() {
    return index_data_->GetData(memory_type_)->size;
  }

  const IndexData* GetIndexData() {
    return index_data_->GetData(memory_type_);
  }

  // supress the default copy constructor and assignment operator
  PlainVoxelArray(const PlainVoxelArray&);
  PlainVoxelArray& operator=(const PlainVoxelArray&); 
};

}
}

#endif
