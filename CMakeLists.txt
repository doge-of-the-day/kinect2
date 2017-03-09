cmake_minimum_required(VERSION 2.8.3)
project(kinect2)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    pcl_ros
)

find_package(freenect2 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${freenect2_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinect2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


add_executable(kinect2_node
    src/kinect2_node.cpp
    src/kinect2_interface.cpp
)

target_link_libraries(kinect2_node
    ${catkin_LIBRARIES}
    ${freenect2_LIBRARY}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

