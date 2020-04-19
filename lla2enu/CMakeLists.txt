cmake_minimum_required(VERSION 2.8.3)
project(lla2enu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geodesy sensor_msgs tf nav_msgs )



add_message_files(
 FILES
 Distance.msg

 )
 
generate_messages(
   DEPENDENCIES
  std_msgs
 )





## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pub src/pub.cpp)
target_link_libraries(pub ${catkin_LIBRARIES})

add_executable(ub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})
add_dependencies(lla2enu custom_messages_generate_messages_cpp)



