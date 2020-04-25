cmake_minimum_required(VERSION 2.8.3)
project(lla2enu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geodesy sensor_msgs tf nav_msgs message_filters)

add_message_files(
 FILES
 Distance.msg

 )
 
generate_messages(
   DEPENDENCIES
  std_msgs
 )
 
 add_service_files(
 FILES
 ComputeDistance.srv
 )


## Declare a catkin package
## catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(CATKIN_DEPENDS nav_msgs message_filters)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(server src/server.cpp)
target_link_libraries(pub ${catkin_LIBRARIES})
add_dependencies(server ${catkin_EXPORTED_TARGETS})

add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})
add_dependencies(sub ${catkin_EXPORTED_TARGETS})

add_dependencies(lla2enu custom_messages_generate_messages_cpp)



