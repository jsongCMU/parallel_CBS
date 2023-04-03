#include <iostream>
#include <fstream>
#include <filesystem>
#include "MAPFLoader.hpp"
#include "AStar.hpp"

int main()
{
    std::string path = "./instances";
    std::string resultFile = "./outputs/result.txt";
    std::string animateFile = "./instances/exp0.txt";
    int testResult = 0;
    for (const auto & entry : std::filesystem::directory_iterator(path)){
        // Directories
        std::string testFile = entry.path();

        // Load MAPF problem
        MAPFLoader loader;
        printf("Test file: %s\n", testFile.c_str());
        MAPFInstance mapfProblem = loader.loadInstanceFromFile(testFile);

        // Create A* solver
        AStar SAPFsolver;

        // Solve Problem
        int failCount=0;
        int sumOfCosts=0;
        std::vector<std::vector<Point2>> paths;
        for(int i = 0; i < mapfProblem.numAgents; i++)
        {
            bool succ = SAPFsolver.solve(mapfProblem, i);
            if(!succ){
                printf("* Failed to solve for agent %d: (%d,%d) -> (%d,%d)\n", i, mapfProblem.startLocs[i].x, mapfProblem.startLocs[i].y, mapfProblem.goalLocs[i].x, mapfProblem.goalLocs[i].y);
                failCount++;
                testResult = 1;
                continue;
            }
            std::vector<Point2> path = SAPFsolver.getPath();
            paths.push_back(path);
            sumOfCosts += path.size();
        }
        printf("SOC = %d\n", sumOfCosts);

        // Log results for specific test file
        if(testFile == animateFile){
            std::ofstream resultStream;
            resultStream.open (resultFile);
            if(resultStream.is_open()){
                // Give input file name
                resultStream << testFile << "\n";
                // Sum of costs
                resultStream << sumOfCosts << "\n";
                // Path of each agent per line
                for(const auto& path: paths)
                {
                    for(const auto& loc: path)
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
    }
    return testResult;

}