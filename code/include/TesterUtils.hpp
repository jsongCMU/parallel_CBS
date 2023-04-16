#ifndef TESTER_UTILS_H
#define TESTER_UTILS_H

#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <chrono>
#include "MAPFInstance.hpp"

// Used if only want to test/evaluate single map; comment out to test all
#define SINGLEFILE ("../instances/maze_map.txt")

void saveToFile(std::string resultFile, std::string mapfFile, const std::vector<std::vector<Point2>> &paths)
{
    std::ofstream resultStream;
    resultStream.open(resultFile, std::fstream::out | std::fstream::trunc);

    int sumOfCosts = 0;
    for (int i = 0; i < paths.size(); i++)
        sumOfCosts += paths[i].size()-1;

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

class TestTimer
{
public:
    inline void start()
    {
        time_ = std::chrono::high_resolution_clock::now();
    };

    inline double elapsed(bool reset=false)
    {
        // Get elapsed time in ms
        std::chrono::duration<double, std::milli> diff = std::chrono::high_resolution_clock::now() - time_;
        if(reset)
            start();
        return diff.count();
    };

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> time_;
};

#endif