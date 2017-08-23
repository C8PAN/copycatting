#include "test_registry.h"
#include "test.h"
#include "test_result.h"
#include <typeinfo>

void TestRegistry::AddTest(Test *test) {
  instance().Add(test);
}

int TestRegistry::RunAllTests(TestResult &result) {
  return instance().Run(result);
}

TestRegistry& TestRegistry::instance() {
  static TestRegistry registry;
  return registry;
}

void TestRegistry::Add(Test *test) {
  if(test==0) {
    tests=test;
    return;
  }

  test->SetNext(tests);
  tests=test;
}

int TestRegistry::Run(TestResult& result) {
  result.StartTests();
  int failure_counter=0;
  for(Test *test=tests; test!=0; test=test->GetNext()) {
    std::cout << "\n launching tests: " << typeid(*test).name() << std::endl;
    test->Run(result);
    if(failure_counter != result.GetFailureCount()) {
      failure_counter = result.GetFailureCount();
      std::cout << "\n TEST -> [FAILURE]\n" << std::endl;
    } else {
      std::cout << "\n TEST -> [OK]\n" << std::endl;
    }
  }
  result.EndTests();
  return result.GetFailureCount();
}
