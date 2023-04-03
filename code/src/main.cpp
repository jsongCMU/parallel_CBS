#include <cstdio>
#include <iostream>
#include <fstream>
#include "MAPFLoader.hpp"
#include "CBSSolver.hpp"

void saveToFile(std::string resultFile, std::string mapfFile, const std::vector<std::vector<Point2>> &paths)
{
    std::ofstream resultStream;
    resultStream.open(resultFile, std::fstream::out | std::fstream::trunc);

    int sumOfCosts = 0;
    for (int i = 0; i < paths.size(); i++)
        sumOfCosts += paths[i].size();

    if (resultStream.is_open())
    {
        // Give input file name
        resultStream << mapfFile << "\n";
        // Sum of costs
        resultStream << sumOfCosts << "\n";
        // Path of each agent per line
        for (const auto &path : paths)
        {
            for (const auto &loc : path)
            {
                resultStream << "(" << loc.x << "," << loc.y << "); ";
            }
            resultStream << "\n";
        }
        resultStream.close();
        printf("Saved paths to %s\n", resultFile.c_str());
    }
    else
    {
        printf("* Could not open result file!\n");
    }
}

int main()
{
    // Load MAPF problem
    MAPFLoader loader;

    std::string resultFile = "../outputs/result.txt";
    std::string fileName = "../instances/exp0.txt";
    MAPFInstance mapfProblem = loader.loadInstanceFromFile(fileName);

    // Run specific version of CBS
    CBSSolver singleThreaded;
    std::vector<std::vector<Point2>> solution = singleThreaded.solve(mapfProblem);

    saveToFile(resultFile, fileName, solution);
}