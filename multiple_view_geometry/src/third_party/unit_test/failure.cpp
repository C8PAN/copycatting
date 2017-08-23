#include "failure.h"
#include <stdio.h>
#include <string.h>

Failure::Failure(const SimpleString &test_name, const SimpleString &file_name,
                 long line_number, const SimpleString &condition) :
  message_(condition), test_name_(test_name), file_name_(file_name), 
  line_number_(line_number)
{}

Failure::Failure(const SimpleString &test_name, const SimpleString &file_name,
                 long line_number, const SimpleString &expected, 
                 const SimpleString &actual) :
  test_name_(test_name), file_name_(file_name), line_number_(line_number) {
  const char *part1 = "expected";
  const char *part3 = " but was";
  char *stage = new char[strlen(part1)+expected.size()+strlen(part3)+
                         actual.size()+1];
  sprintf(stage, "%s%s%s%s", part1, expected.AsCharString(), part3, 
          actual.AsCharString());
  message_= SimpleString(stage);
  delete stage;
}
