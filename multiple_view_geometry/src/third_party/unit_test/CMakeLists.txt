cmake_minimum_required(VERSION 2.8)

project(UnitTest)
FILE(GLOB UnitTest_SRC "*.h" "*.cpp")
ADD_LIBRARY(UnitTest ${UnitTest_SRC})
SET_TARGET_PROPERTIES(UnitTest PROPERTIES LINKER_LANGUAGE CXX)
