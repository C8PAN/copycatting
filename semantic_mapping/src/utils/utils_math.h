#ifndef UTILS_MATH_H_C4IAEA0L
#define UTILS_MATH_H_C4IAEA0L

#ifndef MIN
#define MIN(a,b) ((a<b)? a : b)
#endif /* ifndef MIN(a,b) ((a<b)? a : b) */

#ifndef MAX
#define MAX(a,b) ((a<b)? b : a)
#endif /* ifndef MAX(a,b) ((a<b)? b : a) */

#ifndef ABS
#define ABS(a) ((a<0)? -a : a)
#endif /* ifndef ABS(a) ((a<0)? -a : a) */

#ifndef CLAMP
#define CLAMP(x,a,b) MAX((a), MIN((b),(x)))
#endif /* ifndef CLAMP(x,a,b) MAX((a), MIN((b),(x)))P */

#ifndef PI
#define PI float(3.1415926535897932384626433832795)
#endif /* ifndef PI float(3.1415926535897932384626433832795) */

#ifndef DEGTORAD
#define DEGTORAD float(0.017453292519943295769236907684886) 
#endif /* ifndef DEGTORAD float(0.017453292519943295769236907684886)  */

#ifndef MY_INF
#define MY_INF 0x7f800000
#endif /* ifndef MY_INF 0x7f800000 */

inline bool PortableFinite(float a) {
  volatile float temp = a;
  if(temp != a)
    return false;
  if((temp-a) != 0.0)
    return false;
  return true;
}

inline void MatVecMultiple(const float* A, const float* b, float* x, 
                           int num_rows, int num_cols) {
  for(int r=0; r<num_rows; ++r) {
    float result = 0.0f;
    for(int c=0; c<num_cols; ++c)
      result += A[r*num_cols+c]*b[c];
    x[r] = result; 
  }
}

#endif /* end of include guard: UTILS_MATH_H_C4IAEA0L */
