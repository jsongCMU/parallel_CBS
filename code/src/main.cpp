#include <cstdio>
#include "MAPFLoader.hpp"
#include "CBSSolver.hpp"

int main()
{
    // Load MAPF problem
    MAPFLoader loader;

    std::string fileName = "./instances/test_57.txt";
    MAPFInstance mapfProblem = loader.loadInstanceFromFile(fileName);

    printf("%d\n\n\n", mapfProblem.numAgents);

    // Run specific version of CBS
    CBSSolver singleThreaded;
    singleThreaded.solve(mapfProblem);
}