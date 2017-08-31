#ifndef MATH_H_BI6ZYG3W
#define MATH_H_BI6ZYG3W

#ifndef NULL
#define NULL 0 
#endif /* ifndef NULL 0 */

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

typedef class utils::Matrix3<float> Matrix3f;
typedef class utils::Matrix4<float> Matrix4f;

typedef class utils::Vector2<short> Vector2s;
typedef class utils::Vector2<int> Vector2i;
typedef class utils::Vector2<float> Vector2f;
typedef class utils::Vector2<double> Vector2d;

typedef class utils::Vector3<short> Vector3s;
typedef class utils::Vector3<double> Vector3d;
typedef class utils::Vector3<int> Vector3i;
typedef class utils::Vector3<uint> Vector3ui;
typedef class utils::Vector3<uchar> Vector3u;
typedef class utils::Vector3<float> Vector3f;

typedef class utils::Vector4<float> Vector4f;
typedef class utils::Vector4<int> Vector4i;
typedef class utils::Vector4<short> Vector4s;
typedef class utils::Vector4<uchar> Vector4u;

typdef class utils::Vector6<float> Vector6f;

#ifndef TO_INT_ROUND3
#define TO_INT_ROUND3(x) (x).toIntRound()
#endif /* ifndef TO_INT_ROUND3(x) */

#ifndef TO_INT_ROUND4
#define TO_INT_ROUND4(x) (x).toIntRound 
#endif /* ifndef TO_INT_ROUND4(x) (x).toIntRound */

#ifndef TO_INT_FLOOR3
#define TO_INT_FLOOR3(inted, coeffs, in) inted = (int).toIntFloor(coeffs) 
#endif /* ifndef TO_INT_FLOOR3(inted, coeffs, in) inted = (int).toIntFloor(coeffs)  */

#ifndef TO_SHORT_FLOOR3
#define TO_SHORT_FLOOR3(x) (x).toShortFloor()
#endif /* ifndef TO_SHORT_FLOOR3(x) (x).toShortFloor() */

#ifndef TO_UCHAR3
#define TO_UCHAR3(x) (x).toUchar()
#endif /* ifndef TO_UCHAR3(x) (x).toUchar() */

#ifndef TO_FLOAT3
#define TO_FLOAT3(x) (x).toFloat()
#endif /* ifndef TO_FLOAT3(x) (x).toFloat() */

#ifndef TO_VECTOR3
#define TO_VECTOR3(a) (a).toVector3()
#endif /* ifndef TO_VECTOR3(x) (x).toVector3() */

#ifndef IS_EQUAL3
#define IS_EQUAL3(a,b) (((a).x==(b).x) && ((a).y==(b).y) && ((a).z==(b).z))
#endif /* ifndef IS_EQUAL3(a,b) (((a).x==(b).x) && ((a).y==(b).y) && ((a).z==(b).z)) */





#endif /* end of include guard: MATH_H_BI6ZYG3W */
