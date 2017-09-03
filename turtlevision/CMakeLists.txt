
cmake_minimum_required(VERSION 2.8.0)
project(turtlevision)

find_package(catkin REQUIRED COMPONENTS qt_build roscpp cv_bridge image_transport rospy std_msgs message_generation)
include_directories(${catkin_INCLUDE_DIRS})


add_message_files(
  FILES
  turtlevision_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()



rosbuild_prepare_qt4(QtCore QtGui)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/turtlevision/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(turtlevision ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(turtlevision ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS turtlevision RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

