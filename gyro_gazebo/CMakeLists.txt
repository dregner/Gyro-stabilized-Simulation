cmake_minimum_required(VERSION 2.8.3)
project(gyro_gazebo)

find_package(catkin REQUIRED)

catkin_package()


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp rospy sensor_msgs cv_bridge
        genmsg
        image_transport
        pcl_conversions
        pcl_msgs
        pcl_ros
        std_msgs)

catkin_package(
        CATKIN_DEPENDS
        gazebo_msgs
        roscpp
        sensor_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(read-imu src/imu/read-imu.cpp)
target_link_libraries(read-imu ${catkin_LIBRARIES})
#add_dependencies()