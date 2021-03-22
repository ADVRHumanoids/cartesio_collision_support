cmake_minimum_required(VERSION 3.0)
project(cartesio_collision_support VERSION 1.0)

find_package(cartesian_interface REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

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
