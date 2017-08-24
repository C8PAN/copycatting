#ifndef CORE_DEFINES_H
#define CORE_DEFINES_H



///////////////////////////////////////////////////////////////////////////////
// voxel hashing definition
///////////////////////////////////////////////////////////////////////////////

#define SDF_BLOCK_SIZE 8
#define SDF_BLOCK_SIZE3 512
#define SDF_LOCAL_BLOCK_NUM 0x40000

#define SDF_GLOBAL_BLOCK_NUM 0x120000 // SDF_BUCKET_NUM+SDF_EXCESS_LIST_SIZE
#define SDF_TRANSFER_BLOCK_NUM 0x1000 // max number of blocks transfered in one swap

#define SDF_BUCKET_NUM 0x100000       // should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM
#define SDF_HASH_MASK 0xfffff         // =SDF_BUCKET_NUM-1
#define SDF_EXCESS_LIST_SIZE 0x20000  // to handle collisions, also max offset value

///////////////////////////////////////////////////////////////////////////////
// voxel hashing data structure 
///////////////////////////////////////////////////////////////////////////////

struct HashEntry {
  // position of the corner of the 8x8x8 volume, that identifies the entry
  Vector3s pos;
  // offset in the excess list
  int offset;
  /* pointer to the voxel block array
     - >= 0 identifies an actual allocated entry in the voxel block array 
     - =1 identifies an entry that has been removed (swapped out)
     - <-1 identifies an unallocated block
  */
  int ptr;
};

struct HashSwapState {
  /* 0 - most recent data is on host, data not currently in active memory
     1 - data on host and in active memory, information has not been combined
     2 - most recent data is in active memory, should save this data to host 
  */
  uchar state;
};

struct VoxelRGB_f {
  _CPU_AND_GPU_CODE_ static float SDFInitialValue() { return 1.0f; }
  _CPU_AND_GPU_CODE_ static float SDFValueToFloat(float x) { return x; }
  _CPU_AND_GPU_CODE_ static float SDFFloatToValue(float x) { return x; }

  static const bool has_color_information_ = true;
  float sdf_;
  // number of fused observations that make up sdf
  uchar w_depth_ 
  Vector3u color_;
  uchar w_color_;
  
  _CPU_AND_GPU_CODE_ VoxelRGB_f() {
    sdf_ = SDFInitialValue();
    w_depth_ = 0;
    color = (uchar)0;
    w_color_ = 0;
  }
};

struct VoxelRGB_s {
  _CPU_AND_GPU_CODE_ static short SDFInitialValue() { return 32767; }
  _CPU_AND_GPU_CODE_ static float SDFValueToFloat(float x) { return (float)(x)/32767.0f; }
  _CPU_AND_GPU_CODE_ static short SDFFloatToValue(float x) { return (short)((x)*32767.0f);}

  static const bool has_color_information_ = true;
  short sdf_;
  uchar w_depth_;
  Vector3u color_;
  uchar w_color_;
  
  _CPU_AND_GPU_CODE_ VoxelRGB_s() {
    sdf_ = SDFInitialValue();
    w_depth_ = 0;
    color_ = (uchar)0;
    w_color_ = 0;
  } 
};

struct Voxel_s {
  _CPU_AND_GPU_CODE_ static short SDFInitialValue() { return 32767; }
  _CPU_AND_GPU_CODE_ static float SDFValueToFloat() { return (float)(x)/32767.0f; }
  _CPU_AND_GPU_CODE_ static short SDFFloatToValue() { return (short)((x)*32767.0f); }
  
  static const bool has_color_information_ = false;
  short sdf_;
  uchar w_depth_;
  // padding that may or may not improve performance on certain GPU
  // uchar pad;  

  _CPU_AND_GPU_CODE_ Voxel_s() {
    sdf_ = SDFInitialValue();
    w_depth_ = 0;
  }
};
                 
struct Voxel_f {
  _CPU_AND_GPU_CODE_ static float SDFInitialValue() { return 1.0f; }
  _CPU_AND_GPU_CODE_ static float SDFValueToFloat(float x) { return x; }
  _CPU_AND_GPU_CODE_ static float SDFFloatToValue(float x) { return x; }
 
  static const bool has_color_information_ = false;
  float sdf_;
  uchar w_depth_;

  _CPU_AND_GPU_CODE_ Voxel_f {
    sdf_ = SDFInitialValue();
    w_depth_ = 0;
  }
};

typedef Voxel_s Voxel;

#ifndef TRACKER_ITERATION_TYPE
#define TRACKER_ITERATION_TYPE
typedef enum {
  TRACKER_ITERATION_ROTATION = 1, TRACKER_ITERATION_TRANSLATION = 2,
  TRACKER_ITERATION_BOTH = 3, TRACKER_ITERATION_NONE = 4
} TrackerIterationType;
#endif

                 
                                                                            
#endif
