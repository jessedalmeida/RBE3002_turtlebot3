cmake_minimum_required(VERSION 2.8.3)
project(rbe3002)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


catkin_python_setup()



catkin_package(
  INCLUDE_DIRS src/lab03
  INCLUDE_DIRS src/lab04
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  MakePath.srv
  RobotNav.srv
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rbe3002.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
