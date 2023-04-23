#include <queue>
#include "CBSSolver.hpp"
#include <chrono>

CBSSolver::CBSSolver(MAPFInstance instance)
    : numNodesGenerated(0), lowLevelSolver(instance)
{
    // Setup locks
    for(int i = 0; i < MAXTHREADS; i++)
    {
        omp_init_lock(&pqLocks[i]);
    }

}

std::vector<std::vector<Point2>> CBSSolver::solveParallel(MAPFInstance instance, double runtimeLimitMs)
{
    // Keep track of time
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsedTime;

    // Create priority queue
    std::priority_queue<CTNodeSharedPtr,
                        std::vector<CTNodeSharedPtr>,
                        CTNodeComparator>
        pq[MAXTHREADS];

    CTNodeSharedPtr root = std::make_shared<CTNode>();
    root->paths.resize(instance.numAgents);
    root->id = 0;
    numNodesGenerated++;

    // Create paths for all agents
    for (int i = 0; i < instance.startLocs.size(); i++)
    {
        bool found = lowLevelSolver.solve(i, root->constraintList, root->paths[i]);

        if (!found)
            throw NoSolutionException();
    }

    root->cost = 0;
    detectCollisions(root->paths, root->collisionList);

    pq[0].push(root);

    // Only run until there are at least MAXTHREADS nodes in the Constraints Tree
    while (!pq[0].empty() && pq[0].size() < MAXTHREADS)
    {
        // Check if abort due to timeout
        if(runtimeLimitMs > 0)
        {
            elapsedTime = std::chrono::high_resolution_clock::now() - startTime;
            if(elapsedTime.count() > runtimeLimitMs)
                throw TimeoutException();
        }

        CTNodeSharedPtr cur = pq[0].top();
        pq[0].pop();

        // If no collisions in the node then return solution
        if (cur->collisionList.size() == 0)
        {
            printf("seq pq %d\n", numNodesGenerated);
            return cur->paths;
        }

        // Get first collision and create two nodes (each containing a new plan for the two agents in the collision)
        for (Constraint &c : resolveCollision(cur->collisionList[0]))
        {
            // Add new constraint
            CTNodeSharedPtr child = std::make_shared<CTNode>();
            child->constraintList = cur->constraintList;
            child->constraintList.push_back(c);
            child->paths = cur->paths;

            // Replan only for the agent that has the new constraint
            child->paths[c.agentNum].clear();
            bool success = lowLevelSolver.solve(c.agentNum, child->constraintList, child->paths[c.agentNum]);

            if (success)
            {
                // Update cost and find collisions
                child->cost = computeCost(child->paths);
                detectCollisions(child->paths, child->collisionList);

                // Set id
                child->id = numNodesGenerated++;

                // Add to search queue
                pq[0].push(child);
            }
        }
    }

    // Split nodes across processors
    for (int i = 1; i < MAXTHREADS; i++)
    {
        pq[i].push(pq[0].top());
        pq[0].pop();
    }
    
    if (pq[0].size() != 1)
    {
        throw std::logic_error("Incorrectly split nodes across processors");
    }

    float bestCost = -1;
    bool timeout = false;
    bool solutionFound = false;
    CTNodeSharedPtr best = nullptr;
    
    #pragma omp parallel num_threads(MAXTHREADS)
    {
        while(true)
        {
            while (!pq[omp_get_thread_num()].empty())
            {
                // Check if abort due to timeout
                if(runtimeLimitMs > 0)
                {
                    elapsedTime = std::chrono::high_resolution_clock::now() - startTime;
                    if(elapsedTime.count() > runtimeLimitMs)
                    {
                        timeout = true;
                        break;
                    }
                }

                omp_set_lock(&pqLocks[omp_get_thread_num()]);
                CTNodeSharedPtr cur = pq[omp_get_thread_num()].top();
                pq[omp_get_thread_num()].pop();
                omp_unset_lock(&pqLocks[omp_get_thread_num()]);

                // Skip any nodes that are worse than the best cost
                if (solutionFound && cur->cost >= bestCost)
                {
                    continue;
                }
    
                if (cur->collisionList.size() == 0)
                {
                    printf("prio pq %d\n", numNodesGenerated);

                    // Only save the solution if it is better than the best seen so far
                    if (cur->cost < bestCost || bestCost < 0)
                    {
                        best = cur;
                        bestCost = cur->cost;
                        solutionFound = true;
                    }
                    
                    continue;
                }

                // Get first collision and create two nodes (each containing a new plan for the two agents in the collision)
                for (Constraint &c : resolveCollision(cur->collisionList[0]))
                {
                    // Add new constraint
                    CTNodeSharedPtr child = std::make_shared<CTNode>();
                    child->constraintList = cur->constraintList;
                    child->constraintList.push_back(c);
                    child->paths = cur->paths;

                    // Replan only for the agent that has the new constraint
                    child->paths[c.agentNum].clear();
                    bool success = lowLevelSolver.solve(c.agentNum, child->constraintList, child->paths[c.agentNum]);

                    if (success)
                    {
                        // Update cost and find collisions
                        child->cost = computeCost(child->paths);
                        detectCollisions(child->paths, child->collisionList);

                        // Set id
                        child->id = numNodesGenerated++;

                        int destPq = child->id % MAXTHREADS;
                        int minSize = pq[omp_get_thread_num()].size();

                        // Load balance the priority queues
                        for (int i = 0; i < MAXTHREADS; i++)
                        {
                            if (i == omp_get_thread_num())
                                continue;

                            if (pq[i].size() < minSize)
                            {
                                destPq = i;
                                minSize = pq[i].size();
                            }
                        }

                        // Assign to any of the search queues
                        omp_set_lock(&pqLocks[destPq]);
                        pq[destPq].push(child);
                        omp_unset_lock(&pqLocks[destPq]);
                    }
                }
            }

            if(timeout)
                break;

            bool allAreEmpty = true;
            for (int i = 0; i < MAXTHREADS; i++)
            {
                if (!pq[i].empty())
                {
                    allAreEmpty = false;
                    break;
                }
            }

            // Only finish if all priority queues are empty
            if (allAreEmpty)
            {
                break;
            }
        }
    }

    if(timeout)
        throw TimeoutException();

    if (best == nullptr)
    {
        throw NoSolutionException();
    }
    else
    {
        return best->paths;
    }
}

std::vector<std::vector<Point2>> CBSSolver::solve(MAPFInstance instance, double runtimeLimitMs)
{
    // Keep track of time
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsedTime;

    // Create priority queue
    std::priority_queue<CTNodeSharedPtr,
                        std::vector<CTNodeSharedPtr>,
                        CTNodeComparator>
        pq;

    CTNodeSharedPtr root = std::make_shared<CTNode>();
    root->paths.resize(instance.numAgents);
    root->id = 0;
    numNodesGenerated++;

    // Create paths for all agents
    for (int i = 0; i < instance.startLocs.size(); i++)
    {
        bool found = lowLevelSolver.solve(i, root->constraintList, root->paths[i]);

        if (!found)
            throw NoSolutionException();
    }

    root->cost = 0;
    detectCollisions(root->paths, root->collisionList);

    pq.push(root);

    while (!pq.empty())
    {
        // Check if abort due to timeout
        if(runtimeLimitMs > 0)
        {
            elapsedTime = std::chrono::high_resolution_clock::now() - startTime;
            if(elapsedTime.count() > runtimeLimitMs)
                throw TimeoutException();
        }

        CTNodeSharedPtr cur = pq.top();
        pq.pop();

        // If no collisions in the node then return solution
        if (cur->collisionList.size() == 0)
        {
            return cur->paths;
        }

        // Get first collision and create two nodes (each containing a new plan for the two agents in the collision)
        for (Constraint &c : resolveCollision(cur->collisionList[0]))
        {
            // Add new constraint
            CTNodeSharedPtr child = std::make_shared<CTNode>();
            child->constraintList = cur->constraintList;
            child->constraintList.push_back(c);
            child->paths = cur->paths;

            // printf("New cons = %d (%d,%d) %d @ %d\n", c.agentNum, c.location.first.x, c.location.first.y, c.isVertexConstraint, c.t);

            // Replan only for the agent that has the new constraint
            child->paths[c.agentNum].clear();
            bool success = lowLevelSolver.solve(c.agentNum, child->constraintList, child->paths[c.agentNum]);

            if (success)
            {
                // Update cost and find collisions
                child->cost = computeCost(child->paths);
                detectCollisions(child->paths, child->collisionList);

                // Set id
                child->id = numNodesGenerated++;

                // Add to search queue
                pq.push(child);
            }
        }
    }

    throw NoSolutionException();
}

int inline CBSSolver::computeCost(const std::vector<std::vector<Point2>> &paths)
{
    int result = 0;

    for (int i = 0; i < paths.size(); i++)
    {
        result += paths[i].size() - 1;
    }

    return result;
}

void CBSSolver::detectCollisions(const std::vector<std::vector<Point2>> &paths, std::vector<Collision> &collisionList)
{
    Collision col;
    collisionList.clear();

    // N^2 alg - seems like there should be a better way to detect collisions
    for (int i = 0; i < paths.size() - 1; i++)
    {
        for (int j = i + 1; j < paths.size(); j++)
        {
            if (detectCollision(i, j, paths[i], paths[j], col))
                collisionList.push_back(col);
        }
    }
}

inline bool CBSSolver::detectCollision(int agent1, int agent2, const std::vector<Point2> &pathA, const std::vector<Point2> &pathB, Collision &col)
{
    int maxTime = std::max(pathA.size(), pathB.size());

    for (int t = 0; t < maxTime; t++)
    {
        if (getLocation(pathA, t) == getLocation(pathB, t))
        {
            col = createVertexCollision(agent1, agent2, t, getLocation(pathA, t));
            return true;
        }

        if (getLocation(pathA, t) == getLocation(pathB, t + 1) && getLocation(pathA, t + 1) == getLocation(pathB, t))
        {
            col = createEdgeCollision(agent1, agent2, t + 1, getLocation(pathA, t), getLocation(pathA, t + 1));
            return true;
        }
    }

    return false;
}

inline Point2 CBSSolver::getLocation(const std::vector<Point2> &path, int t)
{
    if (t >= path.size())
        return path[path.size() - 1];
    else
        return path[t];
}

inline std::vector<Constraint> CBSSolver::resolveCollision(const Collision &col)
{
    if (col.isVertexCollision)
    {
        return std::vector<Constraint>{Constraint{col.agent1, col.t, true, col.location}, Constraint{col.agent2, col.t, true, col.location}};
    }
    else
    {
        return std::vector<Constraint>{Constraint{col.agent1, col.t, false, col.location}, Constraint{col.agent2, col.t, false, col.location}};
    }
}