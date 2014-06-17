cmake_minimum_required(VERSION 2.8.3)
project(iri_laser_icp)

# Include csm project code and compile it
# From https://github.com/ccny-ros-pkg/scan_tools/blob/hydro/laser_scan_matcher/csm
include(${CMAKE_CURRENT_SOURCE_DIR}/csm/GetCSM.cmake)
download_and_compile_csm()

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    tf
    sensor_msgs
    geometry_msgs
    nav_msgs
    iri_base_algorithm)

add_service_files(FILES
  GetRelativePose.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs)

generate_dynamic_reconfigure_options(cfg/LaserIcp.cfg)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    tf
    sensor_msgs
    geometry_msgs
    nav_msgs
    iri_base_algorithm)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${csm_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/laser_icp_alg.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LINK_LIBRARIES} gsl ${csm_LIBRARIES})
add_dependencies(${PROJECT_NAME} iri_laser_icp_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${csm_EXPORTED_TARGETS})

add_executable(laser_icp_alg_node src/laser_icp_alg_node.cpp)
target_link_libraries(laser_icp_alg_node ${PROJECT_NAME})

install(PROGRAMS scripts/test_laser_icp.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
