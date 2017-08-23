#ifndef TEST_REGISTRY_H
#define TEST_REGISTRY_H

class Test;
class TestResult;

class TestRegistry {
public:
  static void AddTest(Test *test);
  static int RunAllTests(TestResult &result);

private:
  static TestRegistry &instance();
  void Add(Test *test);
  int Run(TestResult &result);

  Test *tests;
};

#endif
