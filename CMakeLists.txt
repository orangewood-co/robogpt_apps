cmake_minimum_required(VERSION 3.0.2)
project(robogpt_apps)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Declare ROS messages, services and actions ##
# If you have any custom messages, services, or actions, declare them here
# add_message_files(FILES Message1.msg Message2.msg)
# add_service_files(FILES Service1.srv Service2.srv)
# add_action_files(FILES Action1.action Action2.action)
# generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

## catkin specific configuration ##
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robogpt_apps
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/robogpt_apps_node.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install Python scripts from the 'scripts' directory
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  FILES_MATCHING PATTERN "*.py"  # Include only Python scripts
)

# If you have any executables or libraries, use the following:
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation (if you have them)
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# Mark other files for installation (e.g. launch files, bag files, etc.)
# install(FILES myfile1 myfile2 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robogpt_apps.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by python nosetests
# catkin_add_nosetests(test)

