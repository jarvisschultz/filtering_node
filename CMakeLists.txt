cmake_minimum_required(VERSION 2.8.3)
project(filtering_node)
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  puppeteer_msgs
  tf
  tf_conversions
  # bfl
  angles
  cmake_modules)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# BFL
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED bfl)
find_package(PkgConfig)

pkg_check_modules(BFL REQUIRED bfl)

message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS system signals)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )


## add eigen:
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
# add logging functionality
find_library(LOG4CXX_LIBRARY log4cxx)


add_library(mobile_robot src/mobile_robot_model.cpp)
add_library(mobile_robot_ekf src/mobile_robot_filter.cpp)
add_executable(ekf_filter src/ekf_filter.cpp)
target_link_libraries(ekf_filter mobile_robot mobile_robot_ekf ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${Boost_LIBRARIES})


add_executable(multi_ekf_filter src/multi_ekf_filter.cpp)
target_link_libraries(multi_ekf_filter mobile_robot mobile_robot_ekf ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${Boost_LIBRARIES})


add_dependencies(multi_ekf_filter ${catkin_EXPORTED_TARGETS})
add_dependencies(ekf_filter ${catkin_EXPORTED_TARGETS})


catkin_package()
