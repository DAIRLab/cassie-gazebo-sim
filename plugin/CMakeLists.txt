cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(lcm REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(cassie_plugin SHARED src/CassiePlugin.cpp src/udp.c src/udp_lcm_translator.cc)
include_directories(include)
include_directories(../../dairlib/bazel-bin/lcmtypes)
target_link_libraries(cassie_plugin ${GAZEBO_LIBRARIES} ${CMAKE_SOURCE_DIR}/libagilitycassie.a)
target_link_libraries(cassie_plugin ${LCM_NAMESPACE}lcm)

add_executable(cassiectrl example/cassiectrl.c)
target_link_libraries(cassiectrl cassie_plugin)
