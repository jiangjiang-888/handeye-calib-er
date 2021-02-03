cmake_minimum_required(VERSION 2.8.3)
project(ar_pose_estimate)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++0x -DUSE_GSTREAMER_1 -march=core2 -DHAVE_NFT=1 -DUSE_GSTREAMER_1)
#SET(CMAKE_CXX_FLAGS "-std=c++0x")
add_compile_options(-std=c++11 -DUSE_GSTREAMER_1 -march=core2 -DHAVE_NFT=1 -DUSE_GSTREAMER_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosmsg
  tf
  common_pkg
)

find_package(yaml-cpp REQUIRED)

catkin_package()

#message("lib:" lib)
#message("catkin_LIBRARIES:" ${catkin_LIBRARIES})

set(LIB_DIR lib)
set(INC_DIR include)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(
    ${LIB_DIR}
    ${LIB_DIR}/linux-x86_64
)

add_executable(ar_pose_estimate
  src/ar_pose_estimate.cpp
)

#add_executable(ar_pose
#    src/ar_pose.cpp)

#add_executable(ar_pose_control
#    src/ar_pose_control.cpp
#    src/controller.cpp)

#add_dependencies(base_controller_interface_node 
#  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#  ${catkin_EXPORTED_TARGETS}
#)

#target_link_libraries(ar_pose
#    ${catkin_LIBRARIES})

#target_link_libraries(ar_pose_control
#    ${catkin_LIBRARIES}
#    yaml-cpp
#    )

target_link_libraries(ar_pose_estimate
  ${catkin_LIBRARIES}
  ARgsub_lite
  ARvideo
  AR
  ARICP
  AR
  ARUtil
  glut
  GLU
  GL
  X11
  m
  #pthread
  jpeg
  z
  udev
  gstreamer-1.0
  gobject-2.0
  glib-2.0
  dc1394
  raw1394
)

install(TARGETS ar_pose_estimate
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS ar_pose
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(TARGETS ar_pose_control
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY Data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Data/)
