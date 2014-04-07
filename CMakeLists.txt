cmake_minimum_required(VERSION 2.8.3)
project(iri_laser_icp)

#Include csm project code and compile it
# From https://github.com/ccny-ros-pkg/scan_tools/blob/hydro/laser_scan_matcher/csm
include(${CMAKE_CURRENT_SOURCE_DIR}/csm/GetCSM.cmake)
download_and_compile_csm()

## Find catkin macros and libraries
find_package(catkin REQUIRED)
find_package(iriutils REQUIRED)


# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS message_generation tf sensor_msgs geometry_msgs nav_msgs iri_base_algorithm)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(<dependency> REQUIRED)

# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetRelativePose.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs
)

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/LaserIcp.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
  CATKIN_DEPENDS message_generation tf sensor_msgs geometry_msgs nav_msgs iri_base_algorithm
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
  DEPENDS iriutils
)

###########
## Build ##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${csm_INCLUDE_DIRS})
include_directories(${iriutils_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/laser_icp_alg.cpp src/laser_icp_alg_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBRARIES} gsl)
target_link_libraries(${PROJECT_NAME} ${csm_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${csm_EXPORTED_TARGETS})

