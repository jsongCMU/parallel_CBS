#include "MAPFLoader.hpp"
#include "AStar.hpp"

int main()
{
    // Load MAPF problem
    MAPFLoader loader;

    std::string fileName = "./instances/random_map.txt";
    MAPFInstance mapfProblem = loader.loadInstanceFromFile(fileName);

    // Display map
    printf("Map:\n");
    for(int i=0; i<mapfProblem.rows; i++){
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
        for(int j=0; j<path.size(); j++)
            printf("(%d, %d), ", path[j].x, path[j].y);
        printf("\n");
    }
    printf("Result: passed %d/%d\n", mapfProblem.numAgents-failCount, mapfProblem.numAgents);
}