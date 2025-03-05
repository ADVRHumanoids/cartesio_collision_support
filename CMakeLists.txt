cmake_minimum_required(VERSION 3.20)
project(cartesio_collision_support VERSION 2.0)

find_package(cartesian_interface REQUIRED)
find_package(cartesian_interface_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(PCL REQUIRED COMPONENTS   common
                                        filters
                                        surface
                                        search
                                        io)
find_package(rosidl_default_generators REQUIRED)


set(CMAKE_CXX_STANDARD 20)

set(msg_files
  "msg/CollisionState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
)

include_directories(
    ${PCL_INCLUDE_DIRS}
    include
)


add_library(cartesio_collision_support_lib SHARED
    src/Collision.cpp
    src/planning_scene/planning_scene_wrapper.cpp)

# set output name
set_target_properties(cartesio_collision_support_lib PROPERTIES OUTPUT_NAME cartesio_collision_support)

target_link_libraries(cartesio_collision_support_lib
    cartesian_interface::CartesianInterface
    cartesian_interface::CartesianInterfaceSolverOpenSot
    cartesian_interface_ros::CartesianInterfaceRos
    # cartesian_interface::cartesian_interface__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${moveit_ros_planning_interface_LIBRARIES}
    ${PCL_LIBRARIES})



install(TARGETS cartesio_collision_support_lib
    DESTINATION lib)

install(DIRECTORY examples/
    DESTINATION share/${PROJECT_NAME}/examples)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)

add_subdirectory(bindings/python/)
