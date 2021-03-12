cmake_minimum_required(VERSION 3.0)
project(cartesio_collision_support VERSION 1.0)

find_package(cartesian_interface REQUIRED)

add_library(cartesio_collision_support SHARED
    src/Collision.cpp)

target_link_libraries(cartesio_collision_support
    cartesian_interface::CartesianInterface)

install(TARGETS cartesio_collision_support
    DESTINATION lib)
