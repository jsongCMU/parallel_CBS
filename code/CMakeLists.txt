cmake_minimum_required(VERSION 2.8)
project(Parallel-CBS)
include(CTest)
include_directories(include/ )

# Optimization flags
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")

add_executable(parallelCBS src/main.cpp src/MAPFLoader.cpp src/CBSSolver.cpp)

add_executable(TestMAPFLoader test/testMAPFLoader.cpp src/MAPFLoader.cpp)
target_compile_features(TestMAPFLoader PRIVATE cxx_std_17)

add_test(NAME TestMAPFLoader
        COMMAND TestMAPFLoader)