cmake_minimum_required(VERSION 3.0.2)
project(apsrc_udp_rosbridge)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps
    autoware_msgs
    geometry_msgs
    sensor_msgs
    apsrc_msgs
    network_interface
    roscpp
    roslib
    roslint
    std_msgs
)

find_package(catkin REQUIRED ${catkin_deps})
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
)

roslint_cpp()


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# here_to_mabx
add_executable(here_to_mabx src/here_to_mabx.cpp)
target_link_libraries(here_to_mabx ${catkin_LIBRARIES})

# ros_to_mabx
add_executable(ros_to_mabx src/ros_to_mabx.cpp)
target_link_libraries(ros_to_mabx ${catkin_LIBRARIES})

# ros_to_mkx
add_executable(ros_to_mkx src/ros_to_mkx.cpp)
target_link_libraries(ros_to_mkx ${catkin_LIBRARIES})

# mabx_to_ros
add_executable(mabx_to_ros src/mabx_to_ros.cpp)
target_link_libraries(mabx_to_ros ${catkin_LIBRARIES})

# cv_to_ros
add_executable(cv_to_ros src/cv_to_ros.cpp)
target_link_libraries(cv_to_ros ${catkin_LIBRARIES})

############
# Install ##
############

install(TARGETS
    here_to_mabx
    ros_to_mabx
    ros_to_mkx
    mabx_to_ros
    cv_to_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)