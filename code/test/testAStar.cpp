#include "TesterUtils.hpp"
#include "MAPFLoader.hpp"
#include "AStar.hpp"

int main()
{
    std::string path = "../instances";
    std::string resultFile = "../outputs/result.txt";
    std::string animateFile = "../instances/exp0.txt";
    int testResult = 0;
    TestTimer ttimer;
    ttimer.start();
    for (const auto & entry : std::filesystem::directory_iterator(path)){
        // Directories
        std::string testFile = entry.path();

        // Load MAPF problem
        MAPFLoader loader;
        printf("Test file: %s\n", testFile.c_str());
        MAPFInstance mapfProblem = loader.loadInstanceFromFile(testFile);

        // Create A* solver
        AStar SAPFsolver(mapfProblem);

        // Solve Problem
        int failCount=0;
        int sumOfCosts=0;
        std::vector<std::vector<Point2>> paths;
        for(int i = 0; i < mapfProblem.numAgents; i++)
        {
            std::vector<Constraint> constraints;
            std::vector<Point2> path;

            bool succ = SAPFsolver.solve(i, constraints, path);
            if(!succ){
                printf("* Failed to solve for agent %d: (%d,%d) -> (%d,%d)\n", i, mapfProblem.startLocs[i].x, mapfProblem.startLocs[i].y, mapfProblem.goalLocs[i].x, mapfProblem.goalLocs[i].y);
                failCount++;
                testResult = 1;
                continue;
            }
            paths.push_back(path);
            sumOfCosts += path.size();
        }
        printf("SOC = %d\n", sumOfCosts);
        printf("Elapsed time = %f ms\n", ttimer.elapsed(true));

        // Log results for specific test file
        if(testFile == animateFile){
            saveToFile(resultFile, testFile, paths);
            ttimer.elapsed(true);
        }
    }
    return testResult;

}