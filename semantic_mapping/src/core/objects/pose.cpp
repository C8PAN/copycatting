#include "pose.h"
#include <math.h>
#include <stdio.h>

using namespace semantic::objects;

Pose::Pose() { this->SetFrom(0,0,0,0,0,0); }
Pose::Pose(float tx, float ty, float tz, float rx, float ry, float rz) {
  this->SetFrom(tx, ty, tz, rx, ry, rz);
}
Pose::Pose(const float pose[6]) { this->SetFrom(pose); }
Pose::Pose(const Matrix4f& src) { this->SetFrom(src); }
Pose::Pose(const Vector6f& tangent) { this->SetFrom(tangent); }
Pose::Pose(const Pose& src) { this->SetFrom(&src); }

#ifndef M_SQRT1_2                                                               
#define M_SQRT1_2 0.707106781186547524401                                       
#endif                                                                          
                                                                                 
#ifndef M_PI                                                                    
#define M_PI 3.14159265358979323846                                             
#endif                                                                          
                                                                                
#ifndef M_PI_2                                                                  
#define M_PI_2 1.5707963267948966192E0                                          
#endif

void Pose::SetFrom(float tx, float ty, float tz, float rx, float ry, float rz) {
  this->parameters_.each.tx = tx;
  this->parameters_.each.ty = ty;
  this->parameters_.each.tz = tz;
  this->parameters_.each.rx = rx;
  this->parameters_.each.ry = ry;
  this->parameters_.each.rz = rz;
  this->SetModelViewFromParameters();
}

void Poase::SetFrom(const Vector6f& tangent) {
  this->parameters_.each.tx = tangent[0];
  this->parameters_.each.ty = tangent[1];
  this->parameters_.each.tz = tangent[2];
  this->parameters_.each.rx = tangent[3];
  this->parameters_.each.ry = tangent[4];
  this->parameters_.each.rz = tangent[5];
  this->SetModelViewFromParameters();
}

void Pose::SetFrom(const float pose[6]) {
  SetFrom(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
}

void Pose::SetFrom(const Pose* pose) {
  this->parameters_.each.tx = pose->parameters_.each.tx;
  this->parameters_.each.ty = pose->parameters_.each.ty;
  this->parameters_.each.tz = pose->parameters_.each.tz;
  this->parameters_.each.rx = pose->parameters_.each.rx;
  this->parameters_.each.ry = pose->parameters_.each.ry;
  this->parameters_.each.rz = pose->parameters_.each.rz;
  transformation_matrix_ = pose->transformation_matrix_; 
}

Pose::SetModelViewFromParameters() {
  float one_6th = 1.0f/6.0f;
  float one_20th = 1.0f/20.0f;
  Vector3f w; 
  w.x = parameters_.each.rx; 
  w.y = parameters_.each.ry; 
  w.z = parameters_.each.rz;
  Vector3f t;
  t.x = parameters_.each.tx;
  t.y = parameters_.each.ty;
  t.z = parameters_.each.tz;
  float theta_square = dot(w, w);
  float theta = sqrt(theta_square);
  float A, B;
  Matrix3f R;
  Vector3f T;
  Vector3f crossV = cross(w, t);
  if(theta_square < 1e-8f) {
    A = 1.0f - one_6th*theta_square;
    B = 0.5f;
    T.x = t.x+0.5f*corssV.x;
    T.y = t.y+0.5f*corssV.y;
    T.z = t.z+0.5f*crossV.z;
  } 
  else {
    float C;
    if(theta_square < 1e-6) {
      C = one_6th*(1.0f-one_20th*theta_square);
      A = 1.0f-theta_square*C;
      B = 0.5f-0.25f*one_6th*theta_square;
    }
    else {
      float inverse_theta = 1.0f/theta;
      A = sinf(theta)*inverse_theta;
      B = (1.0f-cosf(theta))*inverse_theta*inverse_theta;
      C = (1.0f-A)*inverse_theta*inverse_theta;
    }
    Vector3f cross2 = corss(w, corssV);
    T.x = t.x+B*corssV.x+C*corss2.x;
    T.y = t.y+B*corssV.y+C*corss2.y;
    T.z = t.z+B*corssV.z+C*corss2.z;
  }
  float wx2 = w.x*w.x, wy2 = w.y*w.y, wz2 = w.z*w.z;
  R.m[0 + 3 * 0] = 1.0f - B*(wy2 + wz2);                                      
  R.m[1 + 3 * 1] = 1.0f - B*(wx2 + wz2);                                      
  R.m[2 + 3 * 2] = 1.0f - B*(wx2 + wy2);                                      
                                                                                 
  float a, b;                                                                 
  a = A * w.z, b = B * (w.x * w.y);                                           
  R.m[0 + 3 * 1] = b - a;                                                     
  R.m[1 + 3 * 0] = b + a;                                                     
                                                                                 
  a = A * w.y, b = B * (w.x * w.z);                                           
  R.m[0 + 3 * 2] = b + a;                                                     
  R.m[2 + 3 * 0] = b - a;                                                     
                                                                                 
  a = A * w.x, b = B * (w.y * w.z);                                           
  R.m[1 + 3 * 2] = b - a;                                                     
  R.m[2 + 3 * 1] = b + a;                                                     
                                                                                 
  transformation_matrix_.m[0 + 4*0] = R.m[0 + 3*0]; 
  transformation_matrix_.m[1 + 4*0] = R.m[1 + 3*0]; 
  transformation_matrix_.m[2 + 4*0] = R.m[2 + 3*0];
  transformation_matrix_.m[0 + 4*1] = R.m[0 + 3*1]; 
  transformation_matrix_.m[1 + 4*1] = R.m[1 + 3*1]; 
  transformation_matrix_.m[2 + 4*1] = R.m[2 + 3*1];
  transformation_matrix_.m[0 + 4*2] = R.m[0 + 3*2]; 
  transformation_matrix_.m[1 + 4*2] = R.m[1 + 3*2]; 
  transformation_matrix_.m[2 + 4*2] = R.m[2 + 3*2];
  transformation_matrix_.m[0 + 4*3] = T.v[0]; 
  transformation_matrix_.m[1 + 4*3] = T.v[1]; 
  transformation_matrix_.m[2 + 4*3] = T.v[2];        
  transformation_matrix_.m[3 + 4*0] = 0.0f; 
  transformation_matrix_.m[3 + 4*1] = 0.0f; 
  transformation_matrix_.m[3 + 4*2] = 0.0f; 
  transformation_matrix_.m[3 + 4*3] = 1.0f;  
}

void Pose::SetParametersFromModelView() {
  Vector3f resultRot;
  Matrix3f R = GetRoation();
  Vector3f T = GetTranslation();

  float cos_angle = (R.m00  + R.m11 + R.m22 - 1.0f) * 0.5f;
  resultRot.x = (R.m[2 + 3 * 1] - R.m[1 + 3 * 2]) * 0.5f;
  resultRot.y = (R.m[0 + 3 * 2] - R.m[2 + 3 * 0]) * 0.5f;
  resultRot.z = (R.m[1 + 3 * 0] - R.m[0 + 3 * 1]) * 0.5f;

  float sin_angle_abs = sqrt(dot(resultRot, resultRot));

  if(cos_angle > M_SQRT1_2) {
    if(sin_angle_abs) {
      float p = asinf(sin_angle_abs) / sin_angle_abs;
      resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
    }
  }
  else {
    if(cos_angle > -M_SQRT1_2) {
      float p = acosf(cos_angle) / sin_angle_abs;
      resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
    }
    else {
      float angle = (float)M_PI - asinf(sin_angle_abs);
      float d0 = R.m[0 + 3 * 0] - cos_angle;
      float d1 = R.m[1 + 3 * 1] - cos_angle;
      float d2 = R.m[2 + 3 * 2] - cos_angle;

      Vector3f r2;

      if(fabsf(d0) > fabsf(d1) && fabsf(d0) > fabsf(d2)) { 
        r2.x = d0; 
        r2.y = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; 
        r2.z = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f; 
      } 
      else {
        if(fabsf(d1) > fabsf(d2)) { 
          r2.x = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; 
          r2.y = d1; 
          r2.z = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f; 
        }
        else { 
          r2.x = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f; 
          r2.y = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f; 
          r2.z = d2; 
        }
      }

      if(dot(r2, resultRot) < 0.0f) { 
        r2.x *= -1.0f; r2.y *= -1.0f; r2.z *= -1.0f; 
      }

      r2 = normalize(r2);

      resultRot.x = angle * r2.x; 
      resultRot.y = angle * r2.y; 
      resultRot.z = angle * r2.z;
    }
  }

  float shtot = 0.5f;
  float theta = sqrt(dot(resultRot, resultRot));

  if(theta > 0.00001f) shtot = sinf(theta * 0.5f) / theta;

  Pose halfrotor(0.0f, 0.0f, 0.0f, resultRot.x * -0.5f, resultRot.y * -0.5f, 
                 resultRot.z * -0.5f);

  Vector3f rottrans = halfrotor.GetRoation() * T;

  if(theta > 0.001f) {
    float denom = dot(resultRot, resultRot);
    float param = dot(T, resultRot) * (1 - 2 * shtot) / denom;
		
    rottrans.x -= resultRot.x * param; 
    rottrans.y -= resultRot.y * param; 
    rottrans.z -= resultRot.z * param;
  }
  else {
    float param = dot(T, resultRot) / 24;
    rottrans.x -= resultRot.x * param; 
    rottrans.y -= resultRot.y * param; 
    rottrans.z -= resultRot.z * param;
  }

  rottrans.x /= 2 * shtot; rottrans.y /= 2 * shtot; rottrans.z /= 2 * shtot;

  this->parameters_.each.rx = resultRot.x; 
  this->parameters_.each.ry = resultRot.y; 
  this->parameters_.each.rz = resultRot.z;
  this->parameters_.each.tx = rottrans.x; 
  this->parameters_.each.ty = rottrans.y; 
  this->parameters_.each.tz = rottrans.z; 
}

Pose Pose::exp(const Vector6f& tangent) {
  return Pose(tangent);
}

Pose::MultiplyWith(const Pose* pose) {
  transformation_matrix_ = transformation_matrix_*pose->transformation_matrix_;
  this->SetParametersFromModelView();
}

Matrix3f Pose::GetRoation() const {
  Matrix3f R;
  R.m[0 + 3*0] = transformation_matrix_.m[0 + 4*0]; 
  R.m[1 + 3*0] = transformation_matrix_.m[1 + 4*0]; 
  R.m[2 + 3*0] = transformation_matrix_.m[2 + 4*0];
  R.m[0 + 3*1] = transformation_matrix_.m[0 + 4*1]; 
  R.m[1 + 3*1] = transformation_matrix_.m[1 + 4*1]; 
  R.m[2 + 3*1] = transformation_matrix_.m[2 + 4*1];
  R.m[0 + 3*2] = transformation_matrix_.m[0 + 4*2]; 
  R.m[1 + 3*2] = transformation_matrix_.m[1 + 4*2]; 
  R.m[2 + 3*2] = transformation_matrix_.m[2 + 4*2];
                                                                                 
  return R;   
}

Vector3f Pose::GetTranslation() const {
  Vector3f T;
  T.v[0] = transformation_matrix_.m[0 + 4*3];
  T.v[1] = transformation_matrix_.m[1 + 4*3];
  T.v[2] = transformation_matrix_.m[2 + 4*3];

  return T;
}

void Pose::GetParameters(Vector3f& translation, Vector3f& rotation) {
  translation.x = this->parameters_.each.tx;
  translation.y = this->parameters_.each.ty;
  translation.z = this->parameters_.each.tz;
  rotation.x = this->parameters_.each.rx;
  rotation.y = this->parameters_.each.ry;
  rotation.z = this->parameters_.each.rz;
}

void Pose::SetMatrix(const Matrix4f& src) {
  transformation_matrix_ = src;
  SetParametersFromModelView();
}

void Pose::SetRotation(const Matrix3f& rotation) {
  transformation_matrix_.m[0 + 4*0] = rotation.m[0 + 3*0];   
  transformation_matrix_.m[1 + 4*0] = rotation.m[1 + 3*0];   
  transformation_matrix_.m[2 + 4*0] = rotation.m[2 + 3*0];   
  transformation_matrix_.m[0 + 4*1] = rotation.m[0 + 3*1];   
  transformation_matrix_.m[1 + 4*1] = rotation.m[1 + 3*1];   
  transformation_matrix_.m[2 + 4*1] = rotation.m[2 + 3*1];   
  transformation_matrix_.m[0 + 4*2] = rotation.m[0 + 3*2];   
  transformation_matrix_.m[1 + 4*2] = rotation.m[1 + 3*2];   
  transformation_matrix_.m[2 + 4*2] = rotation.m[2 + 3*2];   
  SetParametersFromModelView();
}

void Pose::SetTranslation(const Vector3f& translation) {
  transformation_matrix_.m[0 + 4*3] = translation.v[0];
  transformation_matrix_.m[1 + 4*3] = translation.v[1];
  transformation_matrix_.m[2 + 4*3] = translation.v[2];
  SetParametersFromModelView();
}

void Pose::SetRotationTranslation(const Matrix3f& rotation, 
                                  const Vector3f& translation) {
  transformation_matrix_.m[0 + 4*0] = rotation.m[0 + 3*0]; 
  transformation_matrix_.m[1 + 4*0] = rotation.m[1 + 3*0]; 
  transformation_matrix_.m[2 + 4*0] = rotation.m[2 + 3*0];
  transformation_matrix_.m[0 + 4*1] = rotation.m[0 + 3*1]; 
  transformation_matrix_.m[1 + 4*1] = rotation.m[1 + 3*1]; 
  transformation_matrix_.m[2 + 4*1] = rotation.m[2 + 3*1];
  transformation_matrix_.m[0 + 4*2] = rotation.m[0 + 3*2]; 
  transformation_matrix_.m[1 + 4*2] = rotation.m[1 + 3*2]; 
  transformation_matrix_.m[2 + 4*2] = rotation.m[2 + 3*2];
  transformation_matrix_.m[0 + 4*3] = translation.v[0]; 
  transformation_matrix_.m[1 + 4*3] = translation.v[1]; 
  transformation_matrix_.m[2 + 4*3] = translation.v[2];        
  SetParametersFromModelView();
}

Matrix4f Pose::GetInverseMatrix() const {
  Matrix4f inverse_matrix;
  transformation_matrix_.inv(inverse_matrix);
  return inverse_matrix;
}

void Pose::SetInverseMatrix(const Matrix4f& inverse_matrix) {
  inverse_matrix.inv(transformation_matrix_);
  SetParametersFromModelView();
}

void Pose::Coerce() {
  SetParametersFromModelView();
  SetModelViewFromParameters();
}
