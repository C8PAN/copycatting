cmake_minimum_required(VERSION 2.8)
project(semantic_mapping)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

macro(CANONIFY_BOOL var)
  if(${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif
endmacro()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIETE)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: " ${libuvc_FOUND})

find_pacakge(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: " ${PNG_FOUND})

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)

set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")

add_subdirectory(utils)
add_subdirectory(lib)
add_subdirectory(engine)

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_executable(semantic_mapping_CLI semantic_mapping_CLI.cpp)
target_link_libraries(semantic_mapping_CLI engine)
target_link_libraries(semantic_mapping_CLI utils)

add_executable(semantic_mapping_GUI semantic_mapping_GUI.cpp)
target_link_libraries(semantic_mapping_GUI engine)
target_link_libraries(semantic_mapping_GUI utils)
