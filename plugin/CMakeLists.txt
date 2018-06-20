cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(cassie_plugin SHARED src/CassiePlugin.cpp src/udp.c)
include_directories(include)
target_link_libraries(cassie_plugin ${GAZEBO_LIBRARIES} ${CMAKE_SOURCE_DIR}/libagilitycassie.a)

add_executable(cassiectrl example/cassiectrl.c)
target_link_libraries(cassiectrl cassie_plugin)
