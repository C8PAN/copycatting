#ifndef TEST_H
#define TEST_H

#include <cmath>
#include "simple_string.h"

class TestResult;

class Test {
public:
  Test(const SimpleString &test_name);
  virtual ~Test() {};
  virtual void Run(TestResult &result)=0;
  void SetNext(Test *test);
  Test *GetNext() const;

protected:
  bool Check(long expected, long actual, TestResult &result, 
             const SimpleString &file_name, long line_number);
  bool Check(const SimpleString &expected, const SimpleString &acutal, 
             TestResult &result, const SimpleString &file_name, 
             long line_number);
  SimpleString name_;
  Test *next_;
};

#define TEST(TestGroup, TestName) \
class TestGroup##_##TestName_Test : public Test { \
public: \
  TestGroup##_##TestName##_##Test() : Test(#TestName "Test") {} \
  void Run(TestResult& result_); \
} TestGroup##_##TestName##_##Instance; \
void TestGroup##_TestName##_##Test::Run(TestResult& result_) 

#define CHECK(condition) { \
  if(!(condition)) { \
    result_.AddFailure(Failure(name_, __FILE__, __LINE__, #condition)); \
    return; \
  } \
}

#define CHECK_EQUAL(expected, actual) { \
  if((expected) != (actual)) \
    result_.AddFailure(Failure(name_, __FILE__, __LINE__, \
                       StringFrom(expected), StringFrom(actual))); \
}

#define LONG_EQUAL(expected, actual) { \
  long actual_temp = actual; \
  long expected_temp = expected; \
  if((expected_temp) != (actual_temp)) { \
    result_.AddFailure(Failure(name_, __FILE__, __LINE__, \
                       StringFrom(expected_temp), StringFrom(actual_temp))); \
    return; \
  } \
}

#define DOUBLES_EQUAL(expected, actual, threshold) { \
  double actual_temp = (double) actual; \
  double expected_temp = (double) expected; \
  if((expected_temp) != (actual_temp)) { \
    result_.AddFailure(Failure(name_, __FILE__, __LINE__, \
                       StringFrom(expected_temp), StringFrom(actual_temp))); \
    return; \
  } \
}

#define FAIL(text) { \
  result_.AddFailure(Failure(name_, __FILE__, __LINE__, (text))); \
  return; \
}

#endif
