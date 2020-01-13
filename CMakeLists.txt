cmake_minimum_required(VERSION 3.15)
project(quickhull)

set(CMAKE_CXX_STANDARD 20)

add_executable(quickhull main.cpp)

if (EMSCRIPTEN)
  set_target_properties(quickhull PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "--emrun"
    SUFFIX ".html"
  )
endif()
