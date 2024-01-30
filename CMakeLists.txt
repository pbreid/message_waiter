cmake_minimum_required(VERSION 3.0.2)
project(message_waiter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  topic_tools
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES message_waiter
  CATKIN_DEPENDS roscpp std_msgs topic_tools
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(message_waiter_node src/message_waiter.cpp)

target_link_libraries(message_waiter_node
  ${catkin_LIBRARIES}
)

install(TARGETS message_waiter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
