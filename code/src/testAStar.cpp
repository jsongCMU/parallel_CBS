#include <iostream>
#include <fstream>
#include "MAPFLoader.hpp"
#include "AStar.hpp"

int main()
{
    // Directories
    std::string testFile = "./instances/random_map.txt";
    std::string resultFile = "./outputs/result.txt";

    // Load MAPF problem
    MAPFLoader loader;
    printf("Test file: %s\n", testFile.c_str());
    MAPFInstance mapfProblem = loader.loadInstanceFromFile(testFile);

    // Display map
    char ruler[] = "0123456789";
    printf("Map (%d,%d):\n", mapfProblem.rows, mapfProblem.cols);
    // Print horizontal ruler
    printf("  ");
    for(int i=0; i<mapfProblem.cols; i++){
        printf("%c ", ruler[i%10]);
    }
    printf("\n");
    // Print map and vertical ruler
    for(int i=0; i<mapfProblem.rows; i++){
        printf("%c ", ruler[i%10]);
        for(int j=0; j<mapfProblem.cols; j++){
            printf("%s", mapfProblem.map[i][j] ? "@ ": ". ");
        }
        printf("\n");
    }
    printf("\n");

    // Create A* solver
    AStar SAPFsolver;

    // Solve Problem
    int failCount=0;
    printf("Solver:\n");
    std::vector<std::vector<Point2>> paths;
    for(int i = 0; i < mapfProblem.numAgents; i++)
    {
        bool succ = SAPFsolver.solve(mapfProblem, i);
        if(!succ){
            printf("* Failed to solve for agent %d: (%d,%d) -> (%d,%d)\n", i, mapfProblem.startLocs[i].x, mapfProblem.startLocs[i].y, mapfProblem.goalLocs[i].x, mapfProblem.goalLocs[i].y);
            failCount++;
            continue;
        }
        printf("Found solution for agent %d:\n\t", i);
        std::vector<Point2> path = SAPFsolver.getPath();
        paths.push_back(path);
        for(int j=0; j<path.size(); j++)
            printf("(%d, %d), ", path[j].x, path[j].y);
        printf("\n");
    }
    printf("Result: passed %d/%d\n", mapfProblem.numAgents-failCount, mapfProblem.numAgents);

    // Log results
    std::ofstream resultStream;
    resultStream.open (resultFile);
    if(resultStream.is_open()){
        // Give input file name
        resultStream << testFile << "\n";
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