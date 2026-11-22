cmake_minimum_required(VERSION 3.14)
project(oriented_field_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs nav_msgs geometry_msgs std_srvs tf2 tf2_ros)
catkin_package()
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

# --- ROS 非依存のマッチングライブラリ ---
add_library(oriented_field_matcher STATIC src/oriented_field_matcher.cpp)
target_include_directories(oriented_field_matcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(oriented_field_matcher PUBLIC ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(oriented_field_matcher PUBLIC OpenMP::OpenMP_CXX)
endif()

add_library(map_yaml_loader STATIC src/map_yaml_loader.cpp)
target_include_directories(map_yaml_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(map_yaml_loader PUBLIC ${OpenCV_LIBS} yaml-cpp)

# --- ROS 1 ノード ---
add_executable(ofl_node src/ofl_node.cpp)
target_include_directories(ofl_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
target_link_libraries(ofl_node oriented_field_matcher map_yaml_loader ${OpenCV_LIBS} ${catkin_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ofl_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS ofl_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
