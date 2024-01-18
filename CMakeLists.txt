cmake_minimum_required(VERSION 2.8.3)
project(ltu_actor_estop)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_srvs
    geometry_msgs
    dbw_polaris_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_srvs
    geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(estop estop.cpp)
target_link_libraries(estop ${catkin_LIBRARIES})
