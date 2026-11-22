cmake_minimum_required(VERSION 3.0.2)

project(mecanum_driver)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    serial
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    serial
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(mecanum_node
    src/mecanum_node.cpp
)

target_link_libraries(mecanum_node
    ${catkin_LIBRARIES}
)

add_dependencies(mecanum_node
    ${catkin_EXPORTED_TARGETS}
)

install(
    TARGETS mecanum_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
