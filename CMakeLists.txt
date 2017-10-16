cmake_minimum_required(VERSION 2.8.3)
project(kinect2)

add_definitions(-std=c++11 -Wall)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -Ofast -march=native")

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    std_srvs
    std_msgs
    pcl_ros
    message_generation
)

find_package(freenect2 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${freenect2_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    include
)

add_message_files(
    FILES
    Kinect2Info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kinect2
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS PCL OpenCV freenect2
)

add_executable(${PROJECT_NAME}_raw_node
    src/kinect2_raw_node.cpp
    src/kinect2_interface.cpp

    include/kinect2/kinect2_node.h
    include/kinect2/kinect2_interface.h
    include/kinect2/kinect2_depth_to_color_map.hpp
)

target_link_libraries(${PROJECT_NAME}_raw_node
    ${catkin_LIBRARIES}
    ${freenect2_LIBRARY}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_raw_node
    ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(${PROJECT_NAME}_node
    src/kinect2_node.cpp
    src/kinect2_interface.cpp

    include/kinect2/kinect2_node.h
    include/kinect2/kinect2_interface.h
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${freenect2_LIBRARY}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
    ${PROJECT_NAME}_generate_messages_cpp
)
