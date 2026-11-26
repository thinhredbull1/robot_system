cmake_minimum_required(VERSION 3.0.2)
project(mecanum_odom_fusion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ekf2d
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs std_srvs tf
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(ekf2d src/ekf2d.cpp)

add_executable(odom_fusion_node src/odom_fusion_node.cpp)
target_link_libraries(odom_fusion_node ekf2d ${catkin_LIBRARIES})
add_dependencies(odom_fusion_node ${catkin_EXPORTED_TARGETS})

install(TARGETS ekf2d odom_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/drive_test.py scripts/evaluate_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_ekf2d test/test_ekf2d.cpp)
  if(TARGET test_ekf2d)
    target_link_libraries(test_ekf2d ekf2d)
  endif()
endif()
