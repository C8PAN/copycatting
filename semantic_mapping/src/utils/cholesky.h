#ifndef CHOLESKY_H
#define CHOLESKY_H

#include <vector>

namespace utils {
	
class Cholesky {
public:
  Cholesky (const float* mat, int size) {
    this->size_ = size;
    this->cholesky_.resize(size*size);
    
    for(int i=0; i<size*size; i++)
      cholesky_[i] = mat[i];
 
    for(int c=0; c<size; c++) {
      float inv_diag = 1;
      for(int r=c; r<size; r++) {
        float val = cholesky_[c+r*size];
        for(int c2=0; c2<c; c2++)
          val -= cholesky_[c+c2*size]*cholesky_[c2+r*size];
        if(r==c) {
          cholesky_[c+r*size] = val;
          if(val == 0)
            rank_ = r;
            inv_diag = 1.0f/val;
        } else {
          cholesky_[r+c*size] = val;
          cholesky_[c+r*size] = val*inv_diag;
        }
      }
    }
    rank_ = size;
  }

  void BackSub(float* result, const float* v) const {
    std::vector<float> y(size_);
    
    for(int i=0; i<size_; i++) {
      float val = v[i];
      for(int j=0; j<i; j++)
        val -= cholesky_[j+i*size_]*y[j];
      y[i] = val;
    }

    for(int i=0; i<size_; i++)
      y[i] /= cholesky_[i+i*size_];

    for(int i=size_-1; i>=0; i--) {
      float val = y[i];
      for(int j=i+1; j<size_; j++)
        val -= cholesky_[i+j*size_]*result[j];
      result[i] = val;
    }
  }

  ~Cholesky() {}

private:
  std::vector<float> cholesky_;
  int size_;
  int rank_;
};
	
} /* utils */ 



#endif /* end of include guard: CHOLESKY_H */
