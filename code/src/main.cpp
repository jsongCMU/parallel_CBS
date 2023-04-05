#include <cstdio>
#include <iostream>
#include <fstream>
#include "MAPFLoader.hpp"
#include "CBSSolver.hpp"
#include "TesterUtils.hpp"

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

    for (int i = 0; i < solution.size(); i++)
    {
        for (int j = 0; j < solution[i].size(); j++)
        {
            printf("(%d,%d) ", solution[i][j].x, solution[i][j].y);
        }

        printf("\n");
    }

    saveToFile(resultFile, fileName, solution);
}