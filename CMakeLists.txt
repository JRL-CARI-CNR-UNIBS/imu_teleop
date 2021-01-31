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
  std_msgs
  moveit_ros_planning_interface
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES imu_teleop
  CATKIN_DEPENDS geometry_msgs ros_myo roscpp sensor_msgs configuration_msgs subscription_notifier std_msgs
  DEPENDS Eigen3
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(imu_teleop_node src/node.cpp)
add_dependencies(imu_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_teleop_node
  ${catkin_LIBRARIES}
)


add_executable(teaching_path_node src/moveit_planning.cpp)
add_dependencies(teaching_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teaching_path_node
  ${catkin_LIBRARIES}
)
