cmake_minimum_required(VERSION 3.0.2)
project(reflector_landmark)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  add_compile_options(/W3)
  set(Boost_USE_STATIC_LIBS FALSE)
else()
  add_compile_options(-Wall -Wextra)
endif()

# Cartographer's exported target references Boost::iostreams, but its installed
# CMake config does not call find_package(Boost) for downstream projects.
# Discover Boost before importing the cartographer target.
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(catkin REQUIRED COMPONENTS
  cartographer_ros_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  xmlrpcpp
)
find_package(cartographer REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reflector_detector reflector_map
  CATKIN_DEPENDS
    cartographer_ros_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    xmlrpcpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(reflector_detector
  src/reflector_detector.cc
)
target_link_libraries(reflector_detector ${catkin_LIBRARIES})
add_dependencies(reflector_detector ${catkin_EXPORTED_TARGETS})

add_library(reflector_map src/reflector_map.cc)

add_executable(reflector_landmark_node
  src/reflector_landmark_node.cc
)
target_link_libraries(reflector_landmark_node
  reflector_detector
  reflector_map
  ${catkin_LIBRARIES}
)
add_dependencies(reflector_landmark_node ${catkin_EXPORTED_TARGETS})

add_executable(reflector_map_exporter
  src/reflector_map_exporter.cc
)
target_link_libraries(reflector_map_exporter cartographer)

if(CATKIN_ENABLE_TESTING)
  add_executable(reflector_regression_test test/reflector_regression_test.cc)
  target_link_libraries(reflector_regression_test
    reflector_detector reflector_map ${catkin_LIBRARIES})
  add_dependencies(reflector_regression_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME reflector_regression_test COMMAND reflector_regression_test)

  catkin_add_gtest(reflector_detector_test
    test/reflector_detector_test.cc
  )
  if(TARGET reflector_detector_test)
    target_link_libraries(reflector_detector_test
      reflector_detector
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS
  reflector_detector
  reflector_map
  reflector_landmark_node
  reflector_map_exporter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config configuration_files launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
