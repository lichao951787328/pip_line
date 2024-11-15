cmake_minimum_required(VERSION 3.0.2)
project(pip_line)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(PYTHON_EXECUTABLE /usr/bin/python3.8)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions(-DOURMETHOD)
find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_msgs
  grid_map_ros
  roscpp
  sensor_msgs
  std_msgs
  diy_msgs
  cv_bridge
  peac
)
set(USING_OURMETHOD OFF)

# set(grid_map_core_DIR "/home/bhr/catkin_gridmap/install/share")
# find_package(grid_map_core REQUIRED)

# set(grid_map_msgs_DIR "/home/bhr/catkin_gridmap/install/share")
# find_package(grid_map_msgs REQUIRED)

# set(grid_map_ros_DIR "/home/bhr/catkin_gridmap/install/share")
# find_package(grid_map_ros REQUIRED)

# set(grid_map_filters_DIR "/home/bhr/catkin_gridmap/install/share")
# find_package(grid_map_filters REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
set(tinyxml2_DIR /usr/local/lib/cmake/tinyxml2)
find_package(tinyxml2 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(PythonLibs REQUIRED)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pip_line
#  CATKIN_DEPENDS grid_map_core grid_map_msgs grid_map_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${grid_map_core_INCLUDE_DIRS}
  # ${grid_map_msgs_INCLUDE_DIRS}
  # ${grid_map_ros_INCLUDE_DIRS}
  # ${grid_map_filters_INCLUDE_DIRS}
  # /home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/include
  # /home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pip_line.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

if(USING_OURMETHOD)
  add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/pip_line.cpp
  src/AstarHierarchicalFootstepPlanner.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_node  PRIVATE OURMETHOD)
else()
  add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/pip_line.cpp
  src/AstarHierarchicalFootstepPlannerTra.cpp
  )
endif()

# add_executable(test_planner 
# test/test_footstep_planner/test.cpp
# src/plane_detection/quatree_node_20240618.cpp
# src/plane_detection/type.cpp
# src/plane_detection/quadtree_new.cpp
# src/plane_detection/plane_new.cpp 
# src/plane_detection/plane_segmentation.cpp 
# src/AstarHierarchicalFootstepPlanner.cpp
# )

# add_executable(test_multi
# test/test_footstep_planner/test_new.cpp
# src/plane_detection/quatree_node_20240618.cpp
# src/plane_detection/type.cpp
# src/plane_detection/quadtree_new.cpp
# src/plane_detection/plane_new.cpp 
# src/plane_detection/plane_segmentation.cpp 
# src/AstarHierarchicalFootstepPlanner.cpp
# )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${grid_map_core_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # ${grid_map_msgs_LIBRARIES}
  # ${grid_map_ros_LIBRARIES}
  # ${grid_map_filters_LIBRARIES}
  glog::glog
  tinyxml2::tinyxml2
)

# target_link_libraries(test_planner
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES} 
#   ${grid_map_core_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   # ${grid_map_msgs_LIBRARIES}
#   # ${grid_map_ros_LIBRARIES}
#   # ${grid_map_filters_LIBRARIES}
#   glog::glog
#   tinyxml2::tinyxml2
# )


# target_link_libraries(test_multi
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES} 
#   ${grid_map_core_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   # ${grid_map_msgs_LIBRARIES}
#   # ${grid_map_ros_LIBRARIES}
#   # ${grid_map_filters_LIBRARIES}
#   glog::glog
#   tinyxml2::tinyxml2
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pip_line.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
