cmake_minimum_required(VERSION 3.0.2)
project(cover)

# Below the various toggles for compilation
option(COVER_BENCHMARK "Toggle to build the benchmark" OFF)
option(COVER_CLANG_TIDY "Toggle to use clang-tidy" OFF)
option(COVER_DOXYGEN "Toggle to build the documentation with Doxygen" OFF)
option(COVER_IWYU "Toggle the include-what-you-use checks" OFF)

set(CATKIN_REQUIRED_PACKAGE costmap_2d roscpp)

# library targets ros-melodic or newer.
# Setting the CXX_STANDARD works only for cmake-versions >= 3.1.0
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wpedantic -Wall -Wextra)

# the clang-tidy checks are optional
if(COVER_CLANG_TIDY)
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy")
endif()

find_package(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_PACKAGE})
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_ros
  CATKIN_DEPENDS ${CATKIN_REQUIRED_PACKAGE}
  DEPENDS EIGEN3 Boost
)

# barebone lib (without ros)
add_library(${PROJECT_NAME}
  src/cover/base.cpp
  src/cover/generators.cpp
  src/cover/sweep.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC src)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ros-interface lib
add_library(${PROJECT_NAME}_ros
  src/cover/costmap.cpp
  src/cover/footprints.cpp
)
target_include_directories(${PROJECT_NAME}_ros PUBLIC src)
target_include_directories(${PROJECT_NAME}_ros SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY src/cover/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # We will compare the results to opencv and ros-navigation's base-local-planner
  find_package(OpenCV REQUIRED)
  find_package(base_local_planner REQUIRED)

  catkin_add_gtest(${PROJECT_NAME}_test
    test/boost_io.cpp
    test/costmap.cpp
    test/expand.cpp
    test/footprints.cpp
    test/generators.cpp
    test/sweep.cpp
  )

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    src
    ${OpenCV_INCLUDE_DIRS}
    ${base_local_planner_INCLUDE_DIRS}
  )
  if (TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test
      ${PROJECT_NAME}
      ${PROJECT_NAME}_ros
      gtest_main
      ${OpenCV_LIBRARIES}
      ${base_local_planner_LIBRARIES}
    )
  endif()
endif()

###############
## Benchmark ##
###############

if(COVER_BENCHMARK)
  find_package(benchmark)
  find_package(base_local_planner REQUIRED)
  find_package(nav_grid_iterators REQUIRED)

  if(benchmark_FOUND)
    add_executable(${PROJECT_NAME}_perf
      perf/expand.cpp
      perf/footprints.cpp
    )
    target_link_libraries(${PROJECT_NAME}_perf
      benchmark::benchmark
      ${PROJECT_NAME}_ros
      ${base_local_planner_LIBRARIES}
      ${nav_grid_iterators_LIBRARIES}
    )
    target_include_directories(${PROJECT_NAME}_perf PRIVATE
      test
      src
      ${base_local_planner_INCLUDE_DIRS}
      ${nav_grid_iterators_INCLUDE_DIRS}
    )

  else()
    message("benchmark library not found")
  endif()
endif()

#############
## Doxygen ##
#############

if(COVER_DOXYGEN)
  find_package(Doxygen REQUIRED)

  if(DOXYGEN_FOUND) 
    set(COVER_DOXYFILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile)

    # Configure the Doxyfile.
    configure_file(doc/Doxyfile.in ${COVER_DOXYFILE_PATH} @ONLY)
    
    # Build the documentation.
    add_custom_target(
      cover_build_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${COVER_DOXYFILE_PATH}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM
    )
  else()
    message("Doxygen not found")
  endif()
endif()

##########
## IWYU ##
##########

if(COVER_IWYU)
  # IWYU is only supported from CMake-version 3.3.0
  if(${CMAKE_VERSION} VERSION_LESS "3.3.0")
    message("Include-what-you-use requires the CMake version 3.3.0")
  else()
    # Follow the documentation of the project to install this on your system.
    # Try to find the executable.
    find_program(iwyu_path NAMES include-what-you-use iwyu)
    if(iwyu_path)
      set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
      set_property(TARGET ${PROJECT_NAME}_ros PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
    else()
      message("include-what-you-use not found")
    endif()
  endif()
endif()
