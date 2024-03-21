cmake_minimum_required(VERSION 3.0)
project(cartesio_collision_support VERSION 1.0)

find_package(cartesian_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(PCL 1.7 QUIET COMPONENTS   common
                                        filters
                                        surface
                                        search
                                        io)
find_package(message_generation)

set(CMAKE_CXX_STANDARD 17)


add_message_files(
  FILES
  CollisionState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()

include_directories(
    ${PCL_INCLUDE_DIRS})

# if(${PCL_FOUND})
#     set(PCM_SRC src/utils/point_cloud_manager.cpp)
# endif()

add_library(cartesio_collision_support SHARED
    src/Collision.cpp
    src/planning_scene/planning_scene_wrapper.cpp
    ${PCM_SRC})

target_link_libraries(cartesio_collision_support
    cartesian_interface::CartesianInterface
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${moveit_ros_planning_interface_LIBRARIES}
    ${PCL_LIBRARIES})

install(TARGETS cartesio_collision_support
    DESTINATION lib)

add_executable(add_sphere examples/add_sphere.cpp)

target_link_libraries(add_sphere
    cartesian_interface::CartesianInterface
    ${moveit_core_LIBRARIES})

# if(${PCL_FOUND})
#     add_executable(point_cloud_node src/utils/point_cloud_node.cpp)

#     target_link_libraries(point_cloud_node cartesio_collision_support)

#     install(TARGETS point_cloud_node
#         DESTINATION lib/${PROJECT_NAME})
# endif()

install(DIRECTORY examples/
    DESTINATION share/${PROJECT_NAME}/examples)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)
