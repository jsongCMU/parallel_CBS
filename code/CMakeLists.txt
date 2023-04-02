cmake_minimum_required(VERSION 2.8)
project(Parallel-CBS)
include_directories(include/ )

# Optimization flags
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")

add_executable(parallelCBS src/main.cpp src/MAPFLoader.cpp src/CBSSolver.cpp)