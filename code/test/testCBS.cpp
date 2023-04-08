#include <algorithm>
#include "TesterUtils.hpp"
#include "MAPFLoader.hpp"
#include "CBSSolver.hpp"

int main()
{
    std::string path = "../instances";
    std::string resultFile = "../outputs/result.txt";
    std::string animateFile = "../instances/exp0.txt";
    std::vector<std::string> ignoreFiles = {
            "../instances/evaluation",
            "../instances/min-sum-of-cost.csv",
            "../instances/maze_map.txt",
            "../instances/random_map.txt",
            "../instances/test_56.txt", // For now skip
            };
    int testResult = 0;
    TestTimer ttimer;
    for (const auto & entry : std::filesystem::directory_iterator(path)){
        // Skip if necessary
        std::string testFile = entry.path();
        if(std::find(ignoreFiles.begin(), ignoreFiles.end(), testFile) != ignoreFiles.end())
            continue;

        // Load MAPF problem
        MAPFLoader loader;
        printf("Test file: %s\n", testFile.c_str());
        MAPFInstance mapfProblem = loader.loadInstanceFromFile(testFile);

        // Create CBS solver
        CBSSolver cbsSolver;

        // Solve Problem
        ttimer.start();
        std::vector<std::vector<Point2>> paths = cbsSolver.solve(mapfProblem);
        double elapsedTime = ttimer.elapsed();

        // Evaluate
        int sumOfCosts=0;
        for(const auto& path : paths)
            sumOfCosts += path.size()-1;
        printf("SOC = %d\n", sumOfCosts);
        printf("Elapsed time = %f ms\n", elapsedTime);

        // Log results for specific test file
        if(testFile == animateFile){
            saveToFile(resultFile, testFile, paths);
        }
    }
    return testResult;

}