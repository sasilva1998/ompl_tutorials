cmake_minimum_required(VERSION 2.8.3)
project(core_octomap_map_representation)


find_package(catkin REQUIRED COMPONENTS
  core_map_representation_interface
  core_trajectory_msgs
  geometry_msgs
  sensor_msgs
  octomap_msgs
  roscpp
  rospy
  tflib
  pcl_ros
  pcl_conversions
  tf
  pluginlib
  visualization_msgs
        )
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(dynamicEDT3D REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES octomap_map_representation
  CATKIN_DEPENDS core_map_representation_interface core_trajectory_msgs octomap_msgs sensor_msgs pcl_ros pcl_conversions roscpp rospy tf visualization_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
        ${DYNAMICEDT3D_INCLUDE_DIRS}
)

 add_library(octomap_map_representation
   src/octomap_map_representation.cpp
 )

target_link_libraries(octomap_map_representation ${catkin_LIBRARIES} ${base_LIBRARIES} ${OCTOMAP_LIBRARIES}  ${PCL_LIBRARIES} dynamicedt3d)


add_executable(octomap_pub src/octomap_pub.cpp)
target_link_libraries(octomap_pub ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(octomap_pub octomap_generate_messages_cpp)

install(TARGETS octomap_map_representation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_core_octomap_map_representation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
