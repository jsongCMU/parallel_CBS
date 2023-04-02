#include "CBSSolver.hpp"
#include <queue>

CBSSolver::CBSSolver()
: numNodesGenerated(0) 
{

}

std::vector<std::vector<Point2>> CBSSolver::solve(MAPFInstance instance)
{
    // Create priority queue
    std::priority_queue <CTNodeSharedPtr, 
                         std::vector<CTNodeSharedPtr>, 
                         CTNodeComparator 
                        > pq;

    CTNodeSharedPtr root = std::make_shared<CTNode>();
    root->cost = 0;
    root->paths.reserve(instance.numAgents);

    // Create paths for all agents
    for (int i = 0; i < instance.startLocs.size(); i++)
    {
        // root->paths.push_back(lowLevelSolver.solve(instance.startLocs[i], instance.goalLocs[i], ));
    }

    pq.push(root);

    while (!pq.empty())
    {
        CTNodeSharedPtr cur = pq.top();
        pq.pop();

        // If no collisions in the node then return solution
        if (cur->collisionList.size() == 0)
        {
            return cur->paths;
        }

        // Get first collision and create two nodes (each containing a new plan for the two agents in the collision)
    }
}