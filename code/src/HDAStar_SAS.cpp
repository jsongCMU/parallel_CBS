#include <queue>
#include <cstdio>
#include <unordered_map>
#include <algorithm>

#include "HDAStar.hpp"

/*
Defines connectivity of neighbourhood
 4 or 8 (+1 for wait) works but 8 is always better
 NOTE: If you change to 4, need to change
 heuristic as well.
*/
#define NBR_CONNECTEDNESS 5

HDAStar::HDAStar(const MAPFInstance& mapfInstance) : AStar(mapfInstance)
{
}

bool HDAStar::solve(const int agent_id, const std::vector<Constraint> &constraints, std::vector<Point2> &outputPath)
{
    Point2 start = _problem.startLocs[agent_id];
    Point2 goal = _problem.goalLocs[agent_id];

    // Create constraints table
    int maxTimestep;
    ConstraintsTable constraintsTable = buildConstraintsTable(constraints, agent_id, maxTimestep);

    // Check validity of start and end
    if (start.x >= _problem.rows || start.y >= _problem.cols)
    {
        printf("* ERR: AStar Failed: Start position not in bounds\n");
        return false;
    }
    else if (goal.x >= _problem.rows || goal.y >= _problem.cols)
    {
        printf("* ERR: AStar Failed: Goal position not in bounds\n");
        return false;
    }

    // Create open lists and visited map
    // Each processor gets its own open list; all processors share visited
    std::priority_queue<LNodeSharedPtr,
                        std::vector<LNodeSharedPtr>,
                        NodeComparator>
        openLists[NUMPROCS];
    std::unordered_map<int, LNodeSharedPtr> visited;
    omp_lock_t visitedLock;
    omp_init_lock(&visitedLock);


    // Create buffers to reduce contention
    // Each processor needs N-1 buffers; have 1 dummy buffer to make it N
    LNodeBuffer openListsBuffer[NUMPROCS][NUMPROCS];
    for(int i=0; i<NUMPROCS; i++)
        for(int j=0; j<NUMPROCS; j++)
            omp_init_lock(&openListsBuffer[i][j].lock);

    // Create flags
    bool finished[NUMPROCS];

    // Create root node
    LNodeSharedPtr root = std::make_shared<LockedNode>();
    root->f = root->g = root->h = 0.0f;
    root->t = 0;
    root->pos = start;
    omp_init_lock(&root->lock);

    // Push to an open list
    openLists[computeDestination(root->pos)].push(root);

    // Possibly suboptimal path found; -1 if none found yet
    float foundPathG = -1;
    int foundPathT = -1;

    while (true)
    {
        // Flush buffer and update open lists
        #pragma omp parallel for
        for(int dstID=0; dstID<NUMPROCS; dstID++)
        {
            for(int srcID=0; srcID<NUMPROCS; srcID++)
            {
                // Grab nodes from srcID, load into dstID
                if(dstID==srcID)
                {
                    // Ignore dummy buffer
                    continue;
                }
                LNodeBuffer &curBuffer = openListsBuffer[dstID][srcID];
                // Claim, copy, clear, release buffer
                omp_set_lock(&curBuffer.lock);
                std::vector<LNodeSharedPtr> curNodes = curBuffer.buffer;
                curBuffer.buffer.clear();
                omp_unset_lock(&curBuffer.lock);

                // Push to open list
                for(const auto& node : curNodes)
                    openLists[dstID].push(node);
            }
        }

        // Check if open list is empty
        #pragma omp parallel for
        for(int pid=0; pid<NUMPROCS; pid++)
        {
            finished[pid] = openLists[pid].empty();
        }

        // Check if all complete
        bool allFinished = true;
        #pragma omp parallel for reduction(&:allFinished)
        for(int pid=0; pid<NUMPROCS; pid++)
        {
            allFinished &= finished[pid];
        }

        if(allFinished)
        {
            int hash = computeHash(goal, foundPathT);
            if (visited.find(hash) != visited.end())
            {
                computePath(visited[hash], outputPath);
                return true;
            }
            else
            {
                // No path to goal found
                return false;
            }
        }


        // Start evaluation
        #pragma omp parallel for // TODO: Fix segfault
        for(int pid=0; pid<NUMPROCS; pid++)
        {
            // Only run if not finished
            if(finished[pid])
                continue;

            // Grab current node
            LNodeSharedPtr cur = openLists[pid].top();
            openLists[pid].pop();

            // If top of priority queue is worse than ceiling, than everything in open list is worse
            // Clear open list
            if(foundPathG > -0.01 && cur->f >= foundPathG)
            {
                while(!openLists[pid].empty())
                    openLists[pid].pop();
                finished[pid] = true;
                continue;
            }

            // Put current node in visisted
            int hash = computeHash(cur->pos, cur->t);


            omp_set_lock(&visitedLock);
            auto x = visited.find(hash);
            omp_unset_lock(&visitedLock);

            if (x != visited.end())
            {
                LNodeSharedPtr existing_node = x->second;
                // Claim, compare, update (if better), release node
                omp_set_lock(&existing_node->lock);
                if (cur->g < existing_node->g)
                {
                    existing_node->g = cur->g;
                    existing_node->f = cur->f;
                    existing_node->t = cur->t;
                    existing_node->parent = cur->parent;
                }
                omp_unset_lock(&existing_node->lock);
            }
            else
            {
                // Node doesn't exist so just add it
                omp_set_lock(&visitedLock);
                visited.insert({hash, cur});
                omp_unset_lock(&visitedLock);
            }

            // If legitimate path found (which may be suboptimal), set/update ceiling
            // All current/neighbor nodes with f value >= ceiling are worse, so ignore
            if(cur->pos == goal && cur->t >= maxTimestep)
            {
                if((foundPathG < 0) || (cur->g < foundPathG))
                {
                    foundPathG = cur->g;
                    foundPathT = cur->t;
                }
            }

            // Generate children
            for (int dir = 0; dir < NBR_CONNECTEDNESS; dir++)
            {
                Point2 nbr_pos = Point2{cur->pos.x + _dx[dir], cur->pos.y + _dy[dir]};

                // Skip if out of bounds
                if (nbr_pos.x >= _problem.rows || nbr_pos.y >= _problem.cols || nbr_pos.x < 0 || nbr_pos.y < 0)
                    continue;

                // Skip if inside obstacle
                if (_problem.map[nbr_pos.x][nbr_pos.y])
                    continue;

                // Skip if violates constraints table
                if (isConstrained(cur->pos, nbr_pos, cur->t + 1, constraintsTable))
                    continue;

                // TODO: Skip if child are worse than what's in visited set? Increases contention for visited set; may be bad idea

                // Create child
                LNodeSharedPtr nbr_node = std::make_shared<LockedNode>();
                nbr_node->pos = nbr_pos;
                nbr_node->g = cur->g + _travel_cost[dir];
                nbr_node->h = _heuristicMap[agent_id][nbr_pos.x][nbr_pos.y];
                nbr_node->f = nbr_node->g + nbr_node->h;
                nbr_node->t = cur->t + 1;
                nbr_node->parent = cur;
                omp_init_lock(&nbr_node->lock);

                // Distribute child
                int destination = computeDestination(nbr_pos);
                if(destination == pid)
                {
                    // Push to open list directly
                    openLists[pid].push(nbr_node);
                }
                else
                {
                    // Claim, push to, release buffer
                    LNodeBuffer &curBuffer = openListsBuffer[destination][pid];
                    omp_set_lock(&curBuffer.lock);
                    curBuffer.buffer.push_back(nbr_node);
                    omp_unset_lock(&curBuffer.lock);
                }
            }
        }
    }

    return false;
}

// Return which processor should get this node
int HDAStar::computeDestination(const Point2& pos)
{
    // Stupid simple for now; refine later
    int nodeNum = pos.x*_problem.cols + pos.y;
    return nodeNum % NUMPROCS;
}
