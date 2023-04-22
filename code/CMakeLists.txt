cmake_minimum_required(VERSION 2.8)
project(Parallel-CBS)
include(CTest)
include_directories(include/)
set (CMAKE_CXX_STANDARD 17) # Use C++17


# Optimization flags
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp")

add_executable(parallelCBS src/main.cpp src/MAPFLoader.cpp src/CBSSolver.cpp src/AStar.cpp)
add_executable(evaluate src/evaluate.cpp src/MAPFLoader.cpp src/CBSSolver.cpp src/AStar.cpp)

add_executable(TestAStar test/testAStar.cpp src/AStar.cpp src/MAPFLoader.cpp)
add_executable(TestMAPFLoader test/testMAPFLoader.cpp src/MAPFLoader.cpp)
add_executable(TestCBS test/testCBS.cpp src/MAPFLoader.cpp src/CBSSolver.cpp src/AStar.cpp)

add_test(NAME TestMAPFLoader
        COMMAND TestMAPFLoader)
add_test(NAME TestAStar
        COMMAND TestAStar)
add_test(NAME TestCBS
        COMMAND TestCBS)
