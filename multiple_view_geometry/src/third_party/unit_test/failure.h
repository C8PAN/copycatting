#ifndef FAILURE_H
#define FAILURE_H

#include "simple_string.h"

class Failure {
public:
  Failure(const SimpleString &test_name, const SimpleString &file_name, 
          long line_number, const SimpleString &condition);
  Failure(const SimpleString &test_name, const SimpleString &file_name, 
          long line_number, const SimpleString &expected, 
          const SimpleString &actual);
  
  SimpleString message_;
  SimpleString test_name_;
  SimpleString file_name_;
  long line_number_;
};

#endif
