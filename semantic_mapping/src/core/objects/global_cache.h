#ifndef GLOBAL_CACHE_H
#define GLOBAL_CACHE_H

#include <stdio.h>
#include <stdlib.h>

#include "defines.h"
#include "../../utils/cuda_wrapper.h"

namespace semantic_mapping {
namespace objects {

template<class TVoxel>
class GlobalCache {
public:
  inline void SetStoredData(int address, TVoxel* data) {
    has_stored_data_[address] = true;
    memcpy(stored_voxel_blocks_+address*SDF_BLOCK_SIZE3, data, 
           sizeof(TVoxel)*SDF_BLOCK_SIZE3);
  }

  inline bool HasStoredData(int address) const {
    return has_stored_data_[address];
  }  

  inline TVoxel* GetStoredVoxelBlock(int address) {
    return stored_voxel_blocks_+address*SDF_BLOCK_SIZE3;
  }

  bool* GetHasSyncedData(bool use_gpu) const {
    return use_gpu ? has_synced_data_device_ : has_synced_data_host_; 
  }

  TVoxel* GetSyncedVoxelBlocks(bool use_gpu) const {
    return use_gpu ? synced_voxel_blocks_device_ : synced_voxel_blocks_host_;
  }  

  HashSwapState* GetSwapStates(bool use_gpu) const {
    return use_gpu ? swap_state_device_ : swap_state_host_;
  } 

  int* GetNeededEntryIds(bool use_gpu) {
    return use_gpu ? needed_entry_ids_device_ : needed_entry_ids_host_;
  }

  int num_total_entries_;

  GlobalCache() : num_total_entries_(SDF_BUCKET_NUM+SDF_EXCESS_LIST_SIZE) {
    has_stored_data_ = (bool*) malloc(num_total_entries_*sizeof(bool));
    stored_voxel_blocks_ 
      = (TVoxel)malloc(num_total_entries_*sizeof(TVoxel)*SDF_BLOCK_SIZE3);
    memset(has_stored_data_, 0, num_total_entries_);
    swap_state_host_ 
      = (HashSwapState*) malloc(num_total_entries_*sizeof(HashSwapState));
    
    SafeCall(cudaMallocHost((void**)&synced_voxel_blocks_host_, 
             SDF_TRANSFER_BLOCK_NUM*sizeof(TVoxel)*SDF_BLOCK_SIZE3)); 
    SafeCall(cudaMallocHost((void**)&has_synced_data_host_,
             SDF_TRANSFER_BLOCK_NUM*sizeof(bool)));
    SafeCall(cudaMallocHost((void**)&needed_entry_ids_host_, 
             SDF_TRANSFER_BLOCK_NUM*sizeof(int)));
    SafeCall(cudaMalloc((void**)&swap_state_device_, 
             num_total_entries_*sizeof(HashSwapState)));    
    SafeCall(cudaMemset(swap_state_device_, 0, 
             num_total_entries_*sizeof(HashSwapState)));
    SafeCall(cudaMalloc((void**)&synced_voxel_blocks_device_, 
             SDF_TRANSFER_BLOCK_NUM*sizeof(TVoxel)*SDF_BLOCK_SIZE3));
    SafeCall(cudaMalloc((void**)&has_synced_data_device_,
             SDF_TRANSFER_BLOCK_NUM*sizeof(bool)));
    SafeCall(cudaMalloc((void**)&needed_entry_ids_device_, 
             SDF_TRANSFER_BLOCK_NUM*sizeof(int)));
  }

  void SaveToFile(char* file_name) const {
    TVoxel* stored_data = stored_voxel_blocks_;
    FILE* f = fopen(file_name, "wb");
    fwrite(has_stored_data_, sizeof(bool), num_total_entries_, f);
    for(int i=0; i<num_total_entries_; i++) {
      fwrite(stored_data, sizeof(TVoxel)*SDF_BLOCK_SIZE3, 1, f);
      stored_data += SDF_BLOCK_SIZE3;
    }
    fclose(f);
  }

  void ReadFromFile(char* file_name) {
    TVoxel* stored_data = stored_voxel_blocks_;
    FILE* f = fopen(file_name, "rb");
    size_t tmp = fread(has_stored_data_, sizeof(bool), num_total_entries_, f);
    if(tmp == (size_t)num_total_entries_) {
      for(int i=0; i<num_total_entries_; i++) {
        fread(stored_data, sizeof(TVoxel)*SDF_BLOCK_SIZE3, 1, f);
        stored_data += SDF_BLOCK_SIZE3;
      }
    }
    fclose(f);
  }

  ~GlobalCache() {
    free(has_stored_data_);
    free(stored_voxel_blocks_);
    free(swap_state_host_);
    
    SafeCall(cudaFreeHost(has_synced_data_host_));
    SafeCall(cudaFree(synced_voxel_blocks_host_));
    SafeCall(cudaFree(needed_entry_ids_host_));
    SafeCall(cudaFree(swap_state_device_));
    SafeCall(cudaFree(synced_voxel_blocks_device_));
    SafeCall(cudaFree(has_synced_data_device_));
    SafeCall(cudaFree(needed_entry_ids_device_));
  }

private:
  bool* has_stored_data_;
  TVoxel* stored_voxel_blocks_;
  HashSwapState* swap_state_host_;
  HashSwapState* swap_state_device_;
  bool* has_synced_data_host_;
  bool* has_synced_data_device_;
  TVoxel* synced_voxel_blocks_host_;
  TVoxel* synced_voxel_blocks_device_;
  int* needed_entry_ids_host_;
  int* needed_entry_ids_device_;
};
		
} /* objects */ 
} /* semantic_mapping */ 


#endif /* end of include guard: GLOBAL_CACHE_H */
