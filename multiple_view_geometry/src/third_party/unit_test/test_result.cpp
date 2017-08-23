#include "test_result.h"
#include "failure.h"

#include <stdio.h>

TestResult::TestResult() : failure_count_(0) {}

void TestResult::StartTests() {}

void TestResult::AddFailure(const Failure &failure) {
  fprintf(stdout, "%s%s%s%s%ld%s%s", "Failure: \"", 
          failure.message_.AsCharString(), "\" ", "line ", 
          failure.line_number_, " in ", failure.file_name_.AsCharString());
  failure_count_++;
}

void TestResult::EndTests() {
  fprintf (stdout, "\n---------\n[TEST SESSION RESULTS]\n---------\n");                                        
  if(failure_count_ > 0)                                                       
    fprintf (stdout, "There were %d failures\n", failure_count_);             
  else                                                                        
    fprintf (stdout, "There were no test failures\n"); 
}
