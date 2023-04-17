#include <algorithm>
#include "TesterUtils.hpp"
#include "MAPFLoader.hpp"
#include "AStar.hpp"

// Answers
std::unordered_map<std::string, int> answers = {
    {"../instances/exp0.txt", 6},
    {"../instances/exp1.txt", 6},
    {"../instances/exp2_1.txt", 6},
    {"../instances/exp2_2.txt", 6},
    {"../instances/exp2_3.txt", 6},
    {"../instances/exp3_1.txt", 6},
    {"../instances/maze_map.txt", 876},
    {"../instances/random_map.txt", 1823},
    {"../instances/test_0.txt", 13},
    {"../instances/test_1.txt", 40},
    {"../instances/test_10.txt", 19},
    {"../instances/test_11.txt", 35},
    {"../instances/test_12.txt", 36},
    {"../instances/test_13.txt", 35},
    {"../instances/test_14.txt", 24},
    {"../instances/test_15.txt", 49},
    {"../instances/test_16.txt", 47},
    {"../instances/test_17.txt", 39},
    {"../instances/test_18.txt", 30},
    {"../instances/test_19.txt", 47},
    {"../instances/test_47.txt", 57},
    {"../instances/test_51.txt", 96},
    {"../instances/test_52.txt", 298},
    {"../instances/test_53.txt", 130},
    {"../instances/test_54.txt", 41},
    {"../instances/test_55.txt", 199},
    {"../instances/test_56.txt", 78},
    {"../instances/test_57.txt", 66},
    {"../instances/test_6.txt", 24},
    {"../instances/test_7.txt", 34},
    {"../instances/test_8.txt", 35},
    {"../instances/exp4.txt", 10},
    {"../instances/test_2.txt", 18},
    {"../instances/test_20.txt", 28},
    {"../instances/test_21.txt", 46},
    {"../instances/test_22.txt", 51},
    {"../instances/test_23.txt", 32},
    {"../instances/test_24.txt", 45},
    {"../instances/test_25.txt", 38},
    {"../instances/test_26.txt", 42},
    {"../instances/test_27.txt", 39},
    {"../instances/test_28.txt", 39},
    {"../instances/test_29.txt", 46},
    {"../instances/test_3.txt", 28},
    {"../instances/test_30.txt", 40},
    {"../instances/test_31.txt", 37},
    {"../instances/test_32.txt", 28},
    {"../instances/test_33.txt", 27},
    {"../instances/test_34.txt", 33},
    {"../instances/test_35.txt", 30},
    {"../instances/test_36.txt", 23},
    {"../instances/test_37.txt", 37},
    {"../instances/test_38.txt", 28},
    {"../instances/test_39.txt", 35},
    {"../instances/test_4.txt", 30},
    {"../instances/test_40.txt", 24},
    {"../instances/test_41.txt", 40},
    {"../instances/test_42.txt", 51},
    {"../instances/test_43.txt", 41},
    {"../instances/test_44.txt", 32},
    {"../instances/test_45.txt", 24},
    {"../instances/test_46.txt", 56},
    {"../instances/test_48.txt", 36},
    {"../instances/test_49.txt", 40},
    {"../instances/test_5.txt", 25},
    {"../instances/test_50.txt", 43},
    {"../instances/test_60.txt", 16},
    {"../instances/test_61.txt", 6},
    {"../instances/test_62.txt", 15},
    {"../instances/test_63.txt", 7},
    {"../instances/test_64.txt", 8},
    {"../instances/test_9.txt", 24}
};

int main()
{
    std::string path = "../instances";
    std::string resultFile = "../outputs/result.txt";
    std::string animateFile = "../instances/exp0.txt";
    std::vector<std::string> ignoreFiles = {
            "../instances/evaluation",
            "../instances/min-sum-of-cost.csv"
    };
    const int testAmount = 5;
    int testResult = 0;
    TestTimer ttimer;
    for (const auto & entry : std::filesystem::directory_iterator(path)){
        // Skip if necessary
        std::string testFile = entry.path();
        #ifdef SINGLEFILE
        animateFile = SINGLEFILE;
        if(testFile != SINGLEFILE)
            continue;
        #endif
        if(std::find(ignoreFiles.begin(), ignoreFiles.end(), testFile) != ignoreFiles.end())
            continue;

        // Load MAPF problem
        MAPFLoader loader;
        printf("Test file: %s\n", testFile.c_str());
        MAPFInstance mapfProblem = loader.loadInstanceFromFile(testFile);

        // Create A* solver
        AStar SAPFsolver(mapfProblem);

        // Solve Problem
        int sumOfCosts;
        double elapsedTime = 0, minTime = 1e30, maxTime = -1e30;
        std::vector<std::vector<Point2>> paths;
        for(int cnt=0; cnt<testAmount; cnt++)
        {
            paths.clear();
            sumOfCosts=0;
            ttimer.start();
            for(int i = 0; i < mapfProblem.numAgents; i++)
            {
                std::vector<Constraint> constraints;
                std::vector<Point2> path;

                bool succ = SAPFsolver.solve(i, constraints, path);
                if(!succ){
                    printf("* Failed to solve for agent %d: (%d,%d) -> (%d,%d)\n", i, mapfProblem.startLocs[i].x, mapfProblem.startLocs[i].y, mapfProblem.goalLocs[i].x, mapfProblem.goalLocs[i].y);
                    testResult = 1;
                    continue;
                }
                paths.push_back(path);
                sumOfCosts += path.size()-1;
            }
            double time = ttimer.elapsed();
            elapsedTime += time;
            minTime = (time < minTime) ? time : minTime;
            maxTime = (time > maxTime) ? time : maxTime;
        }
        elapsedTime /= testAmount;

        // Evaluate
        if (answers.find(testFile) != answers.end()) {
            int expectedSOC = answers[testFile];
            if(expectedSOC != sumOfCosts)
            {
                printf("* Incorrect SOC; expected %d, got %d\n", expectedSOC, sumOfCosts);
                testResult = 1;
            }
            else
                printf("SOC = %d\n", sumOfCosts);
        }
        else
        {
            printf("SOC = %d (not tested)\n", sumOfCosts);
        }
        printf("Elapsed time = %f [%f,%f] ms\n", elapsedTime, minTime, maxTime);

        // Log results for specific test file
        if(testFile == animateFile){
            saveToFile(resultFile, testFile, paths);
        }
    }
    if(testResult)
        printf("* Test failed\n");
    else
        printf("Passed all tests!\n");
    return testResult;

}