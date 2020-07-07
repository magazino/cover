cmake_minimum_required(VERSION 2.8.1)
project(cover)

set(CATKIN_REQUIRED_PACKAGE costmap_2d roscpp)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_PACKAGE})
find_package(Eigen3 REQUIRED)
find_library(GEOS_LIBRARIES geos PATHS /usr/lib /usr/local/lib)
find_path(GEOS_INCLUDE_DIRS geos HINTS /usr/include /usr/local/include)

# barebone lib
add_library(${PROJECT_NAME} src/cover.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src)
target_include_directories(${PROJECT_NAME} PUBLIC SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GEOS_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GEOS_LIBRARIES})

# ros-interface lib
add_library(${PROJECT_NAME}_ros src/cover_ros.cpp)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME})

# test node
add_executable(${PROJECT_NAME}_node src/cover_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})