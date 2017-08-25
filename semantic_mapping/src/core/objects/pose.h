#ifndef POSE_H
#define POSE_H

namespace semantic_mapping {
namespace objects {
	
class Pose {
public:
  void SetFrom(float tx, float ty, float tz, float rx, float ry, float rz);
  void SetFrom(const Vector3f& translation, const Vector3f& rotation);
  void SetFrom(const Vector6f& tangent);
  void SetFrom(const float pose[6]);
  void SetFrom(const Pose* pose);

  // multiply a pose on the right: this=this*pose
  void MultiplyWith(const Pose* pose);
  const Matrix4f& GetMatrix() const { return transformation_matrix_; }
  Matrix3f GetRoation() const;
  Vector3f GetTranslation() const;
  void GetParameters(Vector3f& translation, Vector3f& rotation);
  void SetMatrix(const Matrix4f& matrix);
  void SetRotation(const Matrix3f& roation);
  void SetTranslation(const Vector3f& translation);
  void SetRotationTranslation(const Matrix3f& rotation, 
                             const Vector3f& translation); 
  Matrix4f GetInverseMatrix() const;
  void SetInverseMatrix(const Matrix4f& inverse_matrix);  
  // enforce the orthonoramlity on the rotation matrix
  void Coerce();
  
  Pose(const Pose& src);
  Pose(const Matrix4f& src);
  Pose(float tx, float ty, float tz, float rx, float ry, float rz);
  Pose(const Vector6f& tangent);
  explicit Pose(const float pose[6]);
  Pose();
  static Pose exp(const Vector6f& tangent);

private:
  union { 
    float all[6];
    // rotation parameters are the Lie algebra representation of SO3
    struct { float tx, ty, tz, rx, ry, rz; } each;
  } parameters_;

  Matrix4f transformation_matrix_;
  void SetParametersFromModelView();
  void SetModelViewFromParameters(); 
};
	
} /* objects */ 	
} /* semantic_mapping */ 

#endif /* end of include guard: POSE_H */
