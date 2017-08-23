#ifndef TEST_RESULT_H
#define TEST_RESULT_H

class Failure;

class TestResult {
public:
  TestResult();
  virtual ~TestResult() {};
  virtual void StartTests();
  virtual void AddFailure(const Failure &failure);
  virtual void EndTests();
  int GetFailureCount() const { return failure_count_; }

private:
  int failure_count_;
};

#endif
