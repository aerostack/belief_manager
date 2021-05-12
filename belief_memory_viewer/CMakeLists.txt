##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME belief_memory_viewer)
project(${PROJECT_NAME} C CXX)

add_definitions(-std=c++11)
##############################################################################
# Catkin
##############################################################################
set(BELIEF_MEMORY_VIEWER_CONTROLLER_SOURCE_DIR
        src/controller/source)

set(BELIEF_MEMORY_VIEWER_CONTROLLER_INCLUDE_DIR
        src/controller/include)
set(BELIEF_MEMORY_VIEWER_CONTROLLER_TEST_DIR
 src/controller/test
)


set(BELIEF_MEMORY_VIEWER_HEADER_FILES
        ${EXECUTION_VIEWER_CONTROLLER_INCLUDE_DIR}/*.h
)

set(BELIEF_MEMORY_VIEWER_SOURCE_FILES
        ${BELIEF_MEMORY_VIEWER_CONTROLLER_SOURCE_DIR}/*.cpp

)
set(BELIEF_MEMORY_VIEWER_HEADER_FILES
        ${BELIEF_MEMORY_VIEWER_VIEW_INCLUDE_DIR}/*.h
)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS droneMsgsROS rviz roscpp image_transport  cv_bridge std_msgs geometry_msgs angles pugixml)
catkin_package(INCLUDE_DIRS ${BELIEF_MEMORY_VIEWER_INCLUDE_DIR} ${BELIEF_MEMORY_VIEWER_INCLUDE_DIR} CATKIN_DEPENDS angles pugixml DEPENDS yaml-cpp)

include_directories(${BELIEF_MEMORY_VIEWER_INCLUDE_DIR})
include_directories(${BELIEF_MEMORY_VIEWER_CONTROLLER_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# check required dependencies
find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

##############################################################################
# Qwt library
##############################################################################

FIND_PATH(QWT_INCLUDE_DIR NAMES qwt.h PATHS
  /usr/include
  PATH_SUFFIXES qwt
)

MESSAGE(STATUS "qwt header: ${QWT_INCLUDE_DIR}")

find_library(QWT_LIB NAMES qwt-qt5)

MESSAGE(STATUS "Found Qwt: ${QWT_LIB}")

#set(QWT_LIBRARY "/usr/lib/libqwt-qt5.so.6.1.2")


include_directories(${QWT_INCLUDE_DIR})
link_directories(${QWT_LIBRARY})

##############################################################################
# Qt Environment
##############################################################################

# included via the dependency call in package.xml
find_package(Qt5 COMPONENTS Core Gui Svg OpenGL Widgets PrintSupport REQUIRED)
#include(${QT_USE_FILE})

## to avoid conflict with boost signals it is needed to define QT_NO_KEYWORDS.
add_definitions(-DQT_NO_KEYWORDS)
ADD_DEFINITIONS(${QT_DEFINITIONS})

##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS  src/controller/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/controller/source/*.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")

##############################################################################
# Binaries
##############################################################################


#MESSAGE(STATUS "Found Qt:  ${QT_LIBRARIES} ")
#add_executable(belief_memory_viewer ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
#add_dependencies(belief_memory_viewer ${catkin_EXPORTED_TARGETS})
#target_link_libraries(belief_memory_viewer ${QWT_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} )
#install(TARGETS belief_memory_viewer RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_library(belief_memory_viewer_lib ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
add_dependencies(belief_memory_viewer_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(belief_memory_viewer_lib ${QWT_LIB} ${catkin_LIBRARIES})
target_link_libraries(belief_memory_viewer_lib yaml-cpp)

qt5_use_modules(belief_memory_viewer_lib Core Gui Svg OpenGL Widgets PrintSupport)

add_executable(belief_memory_viewer ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
add_dependencies(belief_memory_viewer ${catkin_EXPORTED_TARGETS})
target_link_libraries(belief_memory_viewer belief_memory_viewer_lib)
target_link_libraries(belief_memory_viewer ${QWT_LIB} ${catkin_LIBRARIES})

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(belief_memory_viewer_test ${BELIEF_MEMORY_VIEWER_CONTROLLER_TEST_DIR}/belief_memory_viewer_test.cpp)
  target_link_libraries(belief_memory_viewer_test belief_memory_viewer_lib)
  target_link_libraries(belief_memory_viewer_test ${catkin_LIBRARIES})
endif()
