#ifndef RENDER_STATE_VOXEL_HASH_H_KNPPUA35
#define RENDER_STATE_VOXEL_HASH_H_KNPPUA35

#include "render_state.h"

namespace semantic_mapping {
namespace objects {

class RenderStateVoxelHash : public RenderState {
public:
  int num_visible_entries_;
  
  RenderStateVoxelHash(int num_total_entries, const Vector2i& image_size, 
                       float vf_min, float vf_max, 
                       MemoryDeviceType memory_type=MEMORY_DEVICE_CPU) 
                       : RenderState(image_size, vf_min, vf_max, memory_type){
    this->memory_type_ = memory_type;
    visible_entry_ids_ 
      = new utils::MemoryBlock<int>(SDF_LOCAL_BLOCK_NUM, memory_type);
    visible_entry_types_ 
      = new utils::MemoryBlock<uchar>(num_total_entries, memory_type);
    num_visible_entries_ = 0;
  }  

  ~RenderStateVoxelHash() {
    delete visible_entry_ids_;
    delete visible_entry_types_;
  }

  const int* GetVisibleEntryIds() const {
    return visible_entry_ids_->GetData(memory_type_);
  } 

  int* GetVisibleEntryIds() {
    return visible_entry_ids_->GetData(memory_type_);
  }

  uchar* GetVisibleEntryTypes() {
    return visible_entry_types_->GetData(memory_type_);
  }

private:
  MemoryDeviceType memory_type_;
  utils::MemoryBlock<int>* visible_entry_ids_;
  utils::MemoryBlock<uchar>* visible_entry_types_;
};
		
} /* objects */ 	
} /* semantic_mapping */ 


#endif /* end of include guard: RENDER_STATE_VOXEL_HASH_H_KNPPUA35 */
