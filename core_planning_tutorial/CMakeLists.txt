cmake_minimum_required(VERSION 2.8.3)
project(core_planning_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 base
 octomap_msgs
 pluginlib
 roscpp
 rospy
 std_msgs
 core_planning_utils
 core_planning_state_space
 core_map_representation_interface

)

find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES core_planning_tutorial
  CATKIN_DEPENDS base octomap_msgs  pluginlib roscpp rospy std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${base_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
)

 add_executable(planning_tutorial src/planning_tutorial.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(planning_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
 target_link_libraries(planning_tutorial
   ${catkin_LIBRARIES}
         ${base_LIBRARIES}
         ${OMPL_LIBRARIES}
 )

 add_executable(planning_tutorial_sp src/planning_tutorial_sp.cpp)

 add_dependencies(planning_tutorial_sp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(planning_tutorial_sp
${catkin_LIBRARIES}
      ${base_LIBRARIES}
      ${OMPL_LIBRARIES}
)