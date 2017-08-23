#include "test.h"
#include "test_registry.h"
#include "test_result.h"
#include "failure.h"

Test::Test(const SimpleString& test_name) : name_(test_name) {
  TestRegistry::AddTest(this);
}

Test* Test::GetNext() const  {
  return next_;
}

void Test::SetNext(Test* test) {
  next_ = test;
}

bool Test::Check(long expected, long actual, TestResult& result, 
                 const SimpleString& file_name, long line_number) {
  if(expected==actual)
    return true;
  result.AddFailure(Failure(name_, StringFrom(__FILE__), __LINE__, 
                    StringFrom(expected), StringFrom(actual)));
  return false;
}

bool Test::Check(const SimpleString& expected, const SimpleString& actual,
                 TestResult& result, const SimpleString& file_name, 
                 long line_number) {
  if(expected==actual)
    return true;
  result.AddFailure(Failure(name_, StringFrom(__FILE__), __LINE__, expected,
                    actual));
  return false;
}  
