#include <algorithm>
#include "TesterUtils.hpp"
#include "MAPFLoader.hpp"
#include "CBSSolver.hpp"

#define MIN_TO_MSEC(t) (t*60*1000)

// Test time limit, in milliseconds
const double testLimit = MIN_TO_MSEC(1);

// Evaluation amount
const int testAmount = 5;

// Evaluation maps
const std::vector<std::string> evalMaps = {
    "../instances/evaluation/evaluation_maze-128-128-10_n10_i0.txt",
    "../instances/evaluation/evaluation_random-32-32-10_n10_i0.txt",
    "../instances/evaluation/evaluation_random-32-32-20_n10_i0.txt",
    "../instances/evaluation/evaluation_room-64-64-16_n10_i0.txt",
    "../instances/evaluation/evaluation_room-64-64-8_n10_i0.txt",
};

// Evaluation output
const std::string evalFile = "../outputs/eval.txt";

int main()
{
    std::ofstream evalStream;
    evalStream.open(evalFile, std::fstream::out | std::fstream::trunc);
    if (!evalStream.is_open())
    {
        printf("* Failed to open logging file: %s\n", evalFile.c_str());
        return 1;
    }
    std::vector<double> speedups;
    speedups.reserve(evalMaps.size());
    for(const std::string &map : evalMaps)
    {
        // Setup
        int prevSOC = -1;
        std::vector<double> serialTimes, parallelTimes;
        serialTimes.resize(testAmount);
        parallelTimes.resize(testAmount);

        MAPFLoader loader;
        std::cout << "Test file: " << map << "\n";
        evalStream << "Test file: " << map << "\n";
        MAPFInstance mapfProblem = loader.loadInstanceFromFile(map);
        CBSSolver cbsSolver(mapfProblem);
        for(int testCnt=0; testCnt<testAmount; testCnt++)
        {
            std::cout << ".";
            std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
            std::chrono::duration<double, std::milli> duration;
            std::vector<std::vector<Point2>> paths;
            int socS=0, socP=0;

            // Solve problem serially
            start = std::chrono::high_resolution_clock::now();
            try
            {
                paths = cbsSolver.solve(mapfProblem, testLimit);
            }
            catch(const CBSSolver::TimeoutException &e)
            {
                printf("* Timeout error!\n");
                return 1;
            }
            catch(...)
            {
                printf("* Invalid MAPF problem!\n");
                return 1;
            }
            end = std::chrono::high_resolution_clock::now();
            duration = end-start;
            serialTimes[testCnt] = duration.count();
            for(const auto &path : paths)
                socS += path.size();
            // Solve problem parallely
            start = std::chrono::high_resolution_clock::now();
            paths = cbsSolver.solveParallel(mapfProblem, testLimit);
            end = std::chrono::high_resolution_clock::now();
            duration = end-start;
            parallelTimes[testCnt] = duration.count();
            for(const auto &path : paths)
                socP += path.size();
            // Verify SOC is same between methods
            if(socS != socP)
            {
                std::cout << "\n";
                std::cout << "* ERRROR! Serial (" << socS << ") and parallel (" << socP << ") have different SOCs!\n";
                evalStream << "* ERRROR! Serial (" << socS << ") and parallel (" << socP << ") have different SOCs!\n";
            }
            // Verify SOC doesn't change between runs
            if(prevSOC < 0)
                prevSOC = socS;
            else if(prevSOC != socS)
            {
                std::cout << "\n";
                std::cout << "* ERRROR! SOC changes between runs! " << prevSOC << " vs " << socS << "\n";
                evalStream << "* ERRROR! SOC changes between runs! " << prevSOC << " vs " << socS << "\n";
            }
        }
        std::cout << "\n";
        // Compute average runtimes and speedups
        float avgS=0, avgP=0, speedup;
        for(int i=0; i<testAmount; i++)
        {
            avgS += serialTimes[i];
            avgP += parallelTimes[i];
        }
        avgS /= testAmount;
        avgP /= testAmount;
        speedup = avgS/avgP;
        // Print
        std::cout << "Avg serial   (ms): " << avgS << "\n";
        std::cout << "Avg parallel (ms): " << avgP << "\n";
        std::cout << "Avg speedup: " << speedup << "\n";
        // Log
        speedups.push_back(speedup);
        evalStream << "\tSpeedup: " << speedup << "\n";
        evalStream << "\tSerial (ms): " << avgS << " (";
        for(int i=0; i<testAmount-1; i++)
            evalStream << serialTimes[i] << ", ";
        evalStream << serialTimes[testAmount-1];
        evalStream << ")\n";
        evalStream << "\tParallel (ms): " << avgP << " (";
        for(int i=0; i<testAmount-1; i++)
            evalStream << parallelTimes[i] << ", ";
        evalStream << parallelTimes[testAmount-1];
        evalStream << ")\n";
    }
    evalStream.close();

    // Summary
    printf("\nSummary:\n");
    for(int i=0; i<evalMaps.size(); i++)
    {
        printf("%s:\t%f\n", evalMaps[i].c_str(), speedups[i]);
    }
    return 0;
}