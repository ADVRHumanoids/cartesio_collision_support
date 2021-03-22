cmake_minimum_required(VERSION 3.0)
project(cartesio_collision_support VERSION 1.0)

find_package(cartesian_interface REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_library(cartesio_collision_support SHARED
    src/Collision.cpp
    src/planning_scene/planning_scene_wrapper.cpp)

target_link_libraries(cartesio_collision_support
    cartesian_interface::CartesianInterface
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${moveit_ros_planning_interface_LIBRARIES})

install(TARGETS cartesio_collision_support
    DESTINATION lib)

add_executable(add_sphere examples/add_sphere.cpp)
target_link_libraries(add_sphere
    cartesian_interface::CartesianInterface
    ${moveit_core_LIBRARIES})

install(DIRECTORY examples/
    DESTINATION share/${PROJECT_NAME}/examples)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)

install(FILES manifest.xml
    DESTINATION share/${PROJECT_NAME})
