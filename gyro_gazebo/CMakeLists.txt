cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gyro_gazebo)

#Adicionando os plugins ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY worlds
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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


#if (${gazebo_VERSION_MAJOR} LESS 6)
#    # These two
#    include(FindBoost)
#    find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#    target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#else()
#    target_link_libraries(vel ${GAZEBO_LIBRARIES})
#endif()

foreach(dir launch models resource worlds)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
