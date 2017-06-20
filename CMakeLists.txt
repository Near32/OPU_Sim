cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(OPUSim_OmniView src/mainOPUSim_OmniView.cpp)
rosbuild_add_executable(OPUSim_HalfView src/mainOPUSim_HalfView.cpp)
rosbuild_add_executable(OPUSim_Obstacles src/mainOPUSim_Obstacles.cpp)
rosbuild_add_executable(OPUSim_RelativeSwarmOdometry src/mainOPUSim_RelativeSwarmOdometry.cpp)
rosbuild_add_executable(OPUSim_ControlLaw src/mainOPUSim_ControlLaw.cpp)
rosbuild_add_executable(OPUSim_ControlLawDynamicObstacle src/mainOPUSim_ControlLawDynamicObstacle.cpp)
rosbuild_add_executable(OPUSim_IMUOdometry src/OPUSim_IMUOdometry/mainOPUSim_IMUOdometry.cpp)
