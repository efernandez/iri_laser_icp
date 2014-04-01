cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(PROJECT_NAME laser_icp_alg_node)

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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# added to include support for dynamic reconfiguration
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
# end dynamic reconfiguration

FIND_PACKAGE(iriutils REQUIRED)

INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR} ./include)

# CSM

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

rosbuild_find_ros_package(csm)

SET(ENV{PKG_CONFIG_PATH} "${csm_PACKAGE_PATH}/lib/pkgconfig")
MESSAGE($ENV{PKG_CONFIG_PATH})

# Require we have pkgconfig installed
find_package(PkgConfig REQUIRED)

# Tell pkgconfig to look for CSM
pkg_check_modules(CSM REQUIRED csm)

IF(${CSM_FOUND})
  MESSAGE("CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
  MESSAGE("CSM_LIBRARIES:    ${CSM_LIBRARIES}")
  MESSAGE("CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")

  include_directories(${CSM_INCLUDE_DIRS})
  link_directories(${CSM_LIBRARY_DIRS})
ELSE(${CSM_FOUND})
  MESSAGE(FATAL_ERROR "CSM not found")
ENDIF(${CSM_FOUND})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/laser_icp_alg.cpp src/laser_icp_alg_node.cpp)

target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY} ${CSM_LIBRARIES})
