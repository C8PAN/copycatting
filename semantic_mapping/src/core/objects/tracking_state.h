#ifndef TRACKING_STATE_H
#define TRACKING_STATE_H

namespace semantic_mapping {
namespace objects {
	
class TrackingState {
public:
  PointCloud* point_cloud_;
  Pose* pose_point_cloud_;
  int age_point_cloud_;
  Pose* pose_depth_;
  bool requires_full_rendering_;
  
  bool TrackerFarFromPointCloud() const {
    if(age_point_cloud_<0)
      return true;
    if(age_point_cloud_>5)
      return true;
  
    Vector3f camera_center_pc = -1.0f*(pose_point_cloud_->GetRotation().t()*
                                       pose_point_cloud_->GetTranslation());
    Vector3f camera_center_live = -1.0f*(pose_depth_->GetRoation().t()*
                                        pose_depth_->GetTranslation());
    Vector3f diff3 = camera_center_pc - camera_center_live;
    float diff = diff3.x*diff3.x + diff3.y*diff3.y + diff3.z*diff3.z;

    if(diff<0.0005f)
      return true;
  
    return false;
  }

  TrackingState(Vector2i image_size, MemoryDeviceType memory_type) {
    this->point_cloud_ = new PointCloud(image_size, memory_type);
    this->pose_depth_ = new Pose();
    this->pose_depth_->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    this->age_point_cloud_ = -1;
    this->pose_point_cloud_ = new Pose();
    this->pose_point_cloud_->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    requires_full_rendering_ = true;
  }  

  ~TrackingState() {
    delete point_cloud_;
    delete pose_depth_;
    delete pose_point_cloud_;
  }

  TrackingState(const TrackingState&);
  TrackingState& operator=(const TrackingState&);
};
	
} /* objects */ 		
} /* semantic_mapping */ 



#endif /* end of include guard: TRACKING_STATE_H */
