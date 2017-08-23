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
  

};

}
}

#endif
