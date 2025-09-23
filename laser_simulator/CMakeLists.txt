cmake_minimum_required(VERSION 3.8)
project(laser_simulator)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-fconcepts -frounding-math ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Armadillo REQUIRED)
find_package(CGAL REQUIRED)
set(CGAL_DATA_DIR ".")
find_package(LAPACK REQUIRED)

# create messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseStampedNamed.msg"
  "msg/PoseStampedNamedArray.msg"
  "msg/ScanPair.msg"
  DEPENDENCIES geometry_msgs sensor_msgs rosidl_default_runtime
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# build the nodes
include_directories(include)

add_library(laser_simlib SHARED src/LaserSimulator.cc)
target_link_libraries(laser_simlib
  ${cpp_typesupport_target}
  ${CGAL_LIBRARY}
  ${ARMADILLO_LIBRARIES}
  ${LAPACK_LIBRARIES}
)
ament_target_dependencies(laser_simlib
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(laser_sim_node src/laser_simulator.cc)
target_link_libraries(laser_sim_node
  ${cpp_typesupport_target}
  laser_simlib
)
ament_target_dependencies(laser_sim_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# ament_export_targets(laser_simlib HAS_LIBRARY_TARGET)
# # Install the library's include directory
# install(DIRECTORY include/my_cpp_library DESTINATION include)

# # Install the library target
# install(TARGETS my_cpp_library
#   EXPORT my_cpp_library
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# add_executable(my_node src/my_node.cpp)
# target_include_directories(my_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(my_node PUBLIC c_std_99 cxx_std_11)  # Require C99 and C++11
# target_link_libraries(my_node PUBLIC pybind11::embed)
# ament_target_dependencies(my_node PUBLIC rclcpp)

install(TARGETS
  laser_simlib
  laser_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
