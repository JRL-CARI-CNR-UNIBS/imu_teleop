cmake_minimum_required(VERSION 3.0.2)
project(imu_teleop)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ros_myo
  roscpp
  sensor_msgs
  configuration_msgs
  subscription_notifier
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES imu_teleop
  CATKIN_DEPENDS geometry_msgs ros_myo roscpp sensor_msgs configuration_msgs subscription_notifier
#  DEPENDS 
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

