#ifndef TESTER_UTILS_H
#define TESTER_UTILS_H

#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include "MAPFInstance.hpp"

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

#endif