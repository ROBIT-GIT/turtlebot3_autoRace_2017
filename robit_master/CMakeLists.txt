cmake_minimum_required(VERSION 2.8.3)
project(robit_master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

#include_directories(${catkin_INCLUDE_DIRS})

 add_message_files(
   FILES
   driving_msg.msg
   
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES robit_master
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
add_library(robit_master src/robitautodv.cpp)

 #add_dependencies(robit_master_node robit_master_generate_message_cpp)
 add_executable(robit_master_node src/robit_master_node.cpp)

 #add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robit_master_node robit_master)
target_link_libraries(robit_master_node ${QT_LIBRARIES} ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
