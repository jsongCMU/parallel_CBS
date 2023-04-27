#include <algorithm>
#include "TesterUtils.hpp"
#include "MAPFLoader.hpp"
#include "HDAStar.hpp"
#include "AStar.hpp"

std::vector<std::string> evalMaps = {
    // "../instances/test_51.txt",
    // "../instances/test_52.txt",
    // "../instances/test_53.txt",
    // "../instances/test_55.txt",
    // "../instances/maze_map.txt",
    // "../instances/random_map.txt",
    // "../instances/eval_coast.txt",
    // "../instances/eval_maze.txt",
    // "../instances/eval_maze2.txt",
    // "../instances/eval_paris.txt",
    // "../instances/eval_random.txt",
    "../instances/eval_room.txt",
};

// Evaluate and compare A* vs HDA*
int main()
{
    std::string path = "../instances";
    std::string resultFile = "../outputs/log.txt";
    std::string animateFile = "../instances/exp0.txt";
    const int testAmount = 1;
    TestTimer ttimer;
    for(int i=0; i<testAmount; i++)
    {
        for (const auto & evalMap : evalMaps){
            // Load MAPF problem
            MAPFLoader loader;
            MAPFInstance mapfProblem = loader.loadInstanceFromFile(evalMap);

            // Create solvers
            HDAStar hdaStarSolver(mapfProblem);
            AStar aStarSolver(mapfProblem);

            // Solve Problem
            int sumOfCosts[2] = {0,0};
            for(int cnt=0; cnt<2; cnt++)
            {
                // Measure
                ttimer.start();
                for(int j = 0; j < mapfProblem.numAgents; j++)
                {
                    std::vector<Point2> path;
                    bool succ;
                    printf("# Agent %d:\n", j);
                    if(cnt==0)
                        succ = aStarSolver.solve(j, {}, path);
                    else
                    {
                        succ = hdaStarSolver.solve(j, {}, path);
                    }
                    if(!succ){
                        printf("* Failed to solve for agent %d: (%d,%d) -> (%d,%d)\n", j, mapfProblem.startLocs[j].x, mapfProblem.startLocs[j].y, mapfProblem.goalLocs[j].x, mapfProblem.goalLocs[j].y);
                        return 1;
                    }
                    sumOfCosts[cnt] += path.size()-1;
                }
                // Display
                printf("%30s: ", evalMap.c_str());
                if(cnt==0)
                    printf("A*   (ms): ");
                else
                    printf("HDA* (ms): ");
                printf("%f\n", ttimer.elapsed());
            }
            if(sumOfCosts[0] != sumOfCosts[1])
            {
                printf("* AStar (%d) and HDAStar (%d) have different SOCs!\n", sumOfCosts[0], sumOfCosts[1]);
                return 1;
            }
        }
    }
    return 0;
}
