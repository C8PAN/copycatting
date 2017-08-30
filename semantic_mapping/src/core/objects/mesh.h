#ifndef MESH_H_IE7YHP3I
#define MESH_H_IE7YHP3I

#include <stdlib.h>

namespace semantic_mapping {
namespace objects {
		
class Mesh {
public:
  struct Triangle { Vector3f p0, p1, p2; }
  MemoryDeviceType memory_type_;
  uint num_total_triangles_;
  static const uint num_max_triangles_ = SDF_LOCAL_BLOCK_NUM*32; 
  utils::MemoryBlock<Triangle>* triangles_;
  
  explicit Mesh(MemoryDeviceType memory_type) {
    this->memory_type_ = memory_type;
    this->num_total_triangles_ = 0;
    triangles_ = new utils::MemoryBlock<Triangle>(num_max_triangles_, 
                                                  memory_type);
  }

  void WriteObj(const char* file_name) {
    utils::MemoryBlock<Triangle>* cpu_triangles;
    bool should_delete = false;
    if(memory_type_ == MEMORY_DEVICE_GPU) {
      cpu_triangles = new utils::MemoryBlock<Triangle>(num_max_triangles_,
                                                       MEMORY_DEVICE_CPU);
      cpu_triangles->SetFrom(triangles_, 
                             utils::MemoryBlock<Triangle>::GPU_TO_CPU);
      should_delete = true;
    }
    else {
      cpu_triangles = triangles_;
    }

    Triangle* triangle_array = cpu_triangles->GetData(MEMORY_DEVICE_CPU);
    FILE* f = fopen(file_name, "w+");
    if(f != NULL) {
      for(uint i=0; i<num_total_triangles_; i++) {
        fprintf(f, "v %f %f %f\n", triangle_array[i].p0.x, 
                triangle_array[i].p0.y, triangle_array[i].p0.z);
        fprintf(f, "v %f %f %f\n", triangle_array[i].p1.x, 
                triangle_array[i].p1.y, triangle_array[i].p1.z);
        fprintf(f, "v %f %f %f\n", triangle_array[i].p2.x, 
                triangle_array[i].p2.y, triangle_array[i].p2.z);
      }
      for(uint i=0; i<num_total_triangles_; i++)
        fprintf(f, "f %d %d %d\n", i*3+3, i*3+2, i*3+1);
      fclose(f);
    } 
    if(should_delete)
      delete cpu_triangles;
  }

  void WriteStl(const char* file_name) {
    utils::MemoryBlock<Triangle> *cpu_triangles;
    bool should_delete = false;
    if(memory_type_ == MEMORY_DEVICE_GPU) {
      cpu_triangles = new utils::MemoryBlock<Triangle>(num_max_triangles_, 
                                                       MEMORY_DEVICE_CPU);
      cpu_triangles->SetFrom(triangles_, 
                             utils::MemoryBlock<Triangle>::GPU_TO_CPU);
      should_delete = true;
    }
    else {
      cpu_triangles = triangles_;
    }
    
    Triangle* triangle_array = cpu_triangles->GetData(MEMORY_DEVICE_CPU);
    FILE* f = fopen(file_name, "wb+");
 
    if(f != NULL) {
      for(int i=0; i<80; i++)
        fwrite(" ", sizeof(char), 1, f);
      fwrite(&num_total_triangles_, sizeof(int), 1, f);
      float zero = 0.0f;
      short attribute = 0;
      for(uint i=0; i<num_total_triangles_; i++) {
        fwrite(&zero, sizeof(float), 1, f);
        fwrite(&zero, sizeof(float), 1, f);
        fwrite(&zero, sizeof(float), 1, f);
      
        fwrite(&triangle_array[i].p2.x, sizeof(float), 1, f);
        fwrite(&triangle_array[i].p2.y, sizeof(float), 1, f);
        fwrite(&triangle_array[i].p2.z, sizeof(float), 1, f);

        fwrite(&triangle_array[i].p1.x, sizeof(float), 1, f);
        fwrite(&triangle_array[i].p1.y, sizeof(float), 1, f);
        fwrite(&triangle_array[i].p1.z, sizeof(float), 1, f);

        fwrite(&triangle_array[i].p0.x, sizeof(float), 1, f);
        fwrite(&triangle_array[i].p0.y, sizeof(float), 1, f);
        fwrite(&triangle_array[i].p0.z, sizeof(float), 1, f);

        fwrite(&attribute, sizeof(short), 1, f);
      }
      fclose(f);
    }     
    if(should_delete) 
      delete triangles_;
  }

  Mesh(const Mesh&);
  Mesh& operator=(const Mesh&);
};

} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: MESH_H_IE7YHP3I */
