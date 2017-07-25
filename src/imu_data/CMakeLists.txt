cmake_minimum_required(VERSION 2.8.3)
project(imu_data)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    mavros
    sensor_msgs
)
catkin_package()
include_directories(
    ${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(imu_data src/imu_data.cpp)
target_link_libraries(imu_data ${catkin_LIBRARIES})
