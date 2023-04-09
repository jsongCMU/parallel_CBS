#include <cstdio>
#include <iostream>
#include <fstream>
#include <chrono>
#include "MAPFLoader.hpp"
#include "CBSSolver.hpp"
#include "TesterUtils.hpp"

int main(int argc, char* argv[])
{
    // Load MAPF problem
    MAPFLoader loader;

    std::string resultFile = "../outputs/result.txt";
    std::string fileName = argv[1];
    MAPFInstance mapfProblem = loader.loadInstanceFromFile(fileName);

    // Run specific version of CBS
    CBSSolver singleThreaded;

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<Point2>> solution = singleThreaded.solve(mapfProblem);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration =  std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    int SOC = 0;
    for(const auto& path : solution)
        SOC += path.size()-1;
    printf("CBS took: %.3fs, SOC = %d\n", duration.count() * 1e-6, SOC);

    saveToFile(resultFile, fileName, solution);
}