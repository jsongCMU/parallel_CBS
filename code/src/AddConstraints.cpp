#include <algorithm>
#include <stdlib.h>
#include "TesterUtils.hpp"
#include "MAPFLoader.hpp"
#include "HDAStar.hpp"

int main()
{
    // Solve for single agent using A* and HDA*, add constraint, repeat
    const std::string testFile = "../instances/eval_room.txt";
    const int constraintLogInterval = 5;
    const int constraintMax = 500;
    const int testAmount = 5;
    const int agentID = 5;
    std::vector<Constraint> constraintAStar, constraintHDAStar;
    // Load problem and solvers
    MAPFLoader loader;
    MAPFInstance mapfProblem = loader.loadInstanceFromFile(testFile);
    HDAStar HDAsolver(mapfProblem);
    AStar Asolver(mapfProblem);
    // Timing
    std::chrono::time_point<std::chrono::high_resolution_clock> time;
    std::chrono::duration<double, std::milli> diff;
    double elapsedTime; // in ms
    // Main loop
    int idx;
    bool doLogging;
    for(int i=0; i<constraintMax; i++)
    {
        /* Logging */
        doLogging = (i%constraintLogInterval) == 0;

        /* AStar */
        std::vector<Point2> pathAStar;
        if(doLogging)
        {
            elapsedTime = 0;
            for(int j=0; j<testAmount; j++)
            {
                pathAStar.clear();
                time = std::chrono::high_resolution_clock::now();
                Asolver.solve(agentID, constraintAStar, pathAStar);
                diff = std::chrono::high_resolution_clock::now() - time;
                elapsedTime += diff.count();
            }
            elapsedTime /= testAmount;
            printf("AStar with %d constraints  : %f ms\n", i, elapsedTime);
        }
        else
            Asolver.solve(agentID, constraintAStar, pathAStar);
        idx = (rand() % (pathAStar.size()-1))+1; // Don't want to pick starting node
        constraintAStar.push_back({agentID, idx, true, {pathAStar[idx], pathAStar[idx]}});
        
        /* HDAStar */
        std::vector<Point2> pathHDAStar;
        if(doLogging)
        {
            elapsedTime = 0;
            for(int j=0; j<testAmount; j++)
            {
                pathHDAStar.clear();
                time = std::chrono::high_resolution_clock::now();
                HDAsolver.solve(agentID, constraintHDAStar, pathHDAStar);
                diff = std::chrono::high_resolution_clock::now() - time;
                elapsedTime += diff.count();
            }
            elapsedTime /= testAmount;
            printf("HDAStar with %d constraints: %f ms\n", i, elapsedTime);
        }
        else
            HDAsolver.solve(agentID, constraintHDAStar, pathHDAStar);
        idx = (rand() % (pathHDAStar.size()-1))+1; // Don't want to pick starting node
        constraintHDAStar.push_back({agentID, idx, true, {pathHDAStar[idx], pathHDAStar[idx]}});
        
    }
    return 0;
}