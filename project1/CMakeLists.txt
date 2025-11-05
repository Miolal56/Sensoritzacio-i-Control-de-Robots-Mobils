cmake_minimum_required(VERSION 3.0.2)
project(project1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  roslib
)


catkin_package()



include_directories(include ${catkin_INCLUDE_DIRS})

## Build map builder
add_executable(world_generator src/world_generator.cpp)

target_link_libraries(world_generator ${catkin_LIBRARIES})

## Build talker and listener
add_executable(talkerGoals src/talkerGoals.cpp)

target_link_libraries(talkerGoals ${catkin_LIBRARIES})

add_executable(moveRobot src/moveRobot.cpp)

target_link_libraries(moveRobot ${catkin_LIBRARIES})
