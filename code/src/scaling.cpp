#include <algorithm>
#include "TesterUtils.hpp"
#include "MAPFGenerator.hpp"
#include "CBSSolver.hpp"

#define MIN_TO_MSEC(t) (t*60*1000)

// Test time limit, in milliseconds
const double testLimit = MIN_TO_MSEC(1);

// Number of times to attempt each map per testing iteration
const int testAmount = 5;

// Number of agents
const int numAgentsInitial = 5;
const int numAgentsIncrement = 5;

// Success threshold; give up below this limit
const double succThresh = 0.5;

// Evaluation maps
const std::vector<std::string> evalMaps = {
    "../instances/evaluation/evaluation_maze-128-128-10.txt",
    "../instances/evaluation/evaluation_random-32-32-10.txt",
    "../instances/evaluation/evaluation_random-32-32-20.txt",
    "../instances/evaluation/evaluation_room-64-64-16.txt",
    "../instances/evaluation/evaluation_room-64-64-8.txt",
};

// Evaluation output
const std::string outputFile = "../outputs/scaling.txt";

int main()
{
    std::ofstream outputStream;
    outputStream.open(outputFile, std::fstream::out | std::fstream::trunc);
    if (!outputStream.is_open())
    {
        printf("* Failed to open logging file: %s\n", outputFile.c_str());
        return 1;
    }

    MAPFGenerator mapfGenerator;
    int numAgents = numAgentsInitial;
    int numTests = evalMaps.size() * testAmount;
    std::cout << "Tests per iteration = " << numTests << "\n";
    outputStream << "Tests per iteration = " << numTests << "\n";
    while(true)
    {
        std::cout << "Number of agents = " << numAgents << "\n";
        outputStream << "Number of agents = " << numAgents << "\n";
        std::vector<int> failuresS, failuresP;
        int numFailuresS=0, numFailuresP=0;
        failuresS.resize(evalMaps.size());
        failuresP.resize(evalMaps.size());
        for(int mapID=0; mapID<evalMaps.size(); mapID++)
        {
            // For each map in evalMaps:
            const std::string &map = evalMaps[mapID];
            for(int testCnt=0; testCnt<testAmount; testCnt++)
            {
                bool checkSOC = true;

                // Generate problem
                MAPFInstance mapfProblem = mapfGenerator.generateProblem(map, numAgents);

                // Create solver
                CBSSolver cbsSolver(mapfProblem);
                std::vector<std::vector<Point2>> paths;
                int socS=0, socP=0;

                // Solve problem serially
                try
                {
                    paths = cbsSolver.solve(mapfProblem, testLimit);
                }
                catch(const CBSSolver::TimeoutException &e)
                {
                    // Failed to solve in time
                    failuresS[mapID]++;
                    numFailuresS++;
                    checkSOC = false;
                }
                catch(...)
                {
                    printf("* Invalid MAPF problem!\n");
                    outputStream.close();
                    return 1;
                }
                for(const auto &path : paths)
                    socS += path.size();

                // Solve problem parallely
                try
                {
                    paths = cbsSolver.solveParallel(mapfProblem, testLimit);
                }
                catch(const CBSSolver::TimeoutException &e)
                {
                    // Failed to solve in time
                    failuresP[mapID]++;
                    numFailuresP++;
                    checkSOC = false;
                }
                catch(...)
                {
                    printf("* Invalid MAPF problem!\n");
                    outputStream.close();
                    return 1;
                }
                for(const auto &path : paths)
                    socP += path.size();

                // Verify SOC is same between methods
                if(checkSOC && socS != socP)
                {
                    std::cout << "* ERRROR! Serial (" << socS << ") and parallel (" << socP << ") have different SOCs!\n";
                    outputStream << "* ERRROR! Serial (" << socS << ") and parallel (" << socP << ") have different SOCs!\n";
                    outputStream << "\tMap: " << map << "\n";
                    outputStream << "\tStart-Goal: ";
                    for(int i=0; i<mapfProblem.numAgents; i++)
                        outputStream << "(" << mapfProblem.startLocs[i].x << "," << mapfProblem.startLocs[i].y << ")-(" << mapfProblem.goalLocs[i].x << "," << mapfProblem.goalLocs[i].y << "); ";
                    outputStream << "\n";
                    outputStream.close();
                    return 1;
                }
            }
        }
        // Calculate success rate
        double succS = (double) (numTests-numFailuresS) / (double) numTests;
        double succP = (double) (numTests-numFailuresP) / (double) numTests;

        // Print/log
        std::cout << "\tSerial:   " << succS << "\n";
        std::cout << "\tParallel: " << succP << "\n";

        outputStream << "\tSerial:   " << succS << " (";
        for(int i=0; i<evalMaps.size()-1; i++)
            outputStream << failuresS[i] << ", ";
        outputStream << failuresS[evalMaps.size()-1] << ")\n";
        
        outputStream << "\tParallel: " << succP << " (";
        for(int i=0; i<evalMaps.size()-1; i++)
            outputStream << failuresP[i] << ", ";
        outputStream << failuresP[evalMaps.size()-1] << ")\n";

        // Terminate if too many failures
        if(succS < succThresh && succP < succThresh)
            break;
        
        // Make problem harder
        numAgents += numAgentsIncrement;
    }
    outputStream.close();
    return 0;
}