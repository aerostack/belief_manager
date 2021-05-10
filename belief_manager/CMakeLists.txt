cmake_minimum_required(VERSION 2.8.3)
project(belief_manager)


## ================= Compiler-specific C++11 activation =================

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
    message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.7 or greater.")
  endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
else ()
  message(FATAL_ERROR "Your C++ compiler does not support C++11.")
endif ()

add_definitions(-std=c++11)

## ================= Set project variables ==================

set(BELIEF_MANAGER_SOURCE_DIR src/source)

set(BELIEF_MANAGER_INCLUDE_DIR src/include)

set(BELIEF_MANAGER_TEST_DIR src/test)

file(GLOB BELIEF_MANAGER_SOURCE_FILES
  "${BELIEF_MANAGER_SOURCE_DIR}/*.cpp"
)

file(GLOB BELIEF_MANAGER_HEADER_FILES
  "${BELIEF_MANAGER_INCLUDE_DIR}/*.h"
)

file(GLOB BELIEF_MANAGER_TEST_FILES
  "${BELIEF_MANAGER_TEST_DIR}/*.cpp"
)

set(LDFLAGS
)

## ================= Find catkin macros and libraries =================

find_package(catkin REQUIRED COMPONENTS
)

## ================= Catkin specific configuration =================

catkin_package(
  INCLUDE_DIRS ${BELIEF_MANAGER_INCLUDE_DIR}
  LIBRARIES belief_manager
)

## ================= Build =================

include_directories(${BELIEF_MANAGER_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(
  belief_manager
  ${BELIEF_MANAGER_SOURCE_FILES}
  ${BELIEF_MANAGER_HEADER_FILES}
)
add_dependencies(belief_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(belief_manager ${catkin_LIBRARIES} ${LDFLAGS})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(belief_manager_test ${BELIEF_MANAGER_TEST_FILES})
  target_link_libraries(belief_manager_test belief_manager)
  target_link_libraries(belief_manager_test ${catkin_LIBRARIES})
endif()
