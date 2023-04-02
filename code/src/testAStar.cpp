#include <cstdio>
#include "MAPFLoader.hpp"
#include "AStar.hpp"

int main()
{
    // Load MAPF problem
    MAPFLoader loader;

    std::string fileName = "./instances/random_map.txt";
    MAPFInstance mapfProblem = loader.loadInstanceFromFile(fileName);

    // Run specific version of CBS
}