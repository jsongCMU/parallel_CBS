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
    std::priority_queue<NodeSharedPtr,
                        std::vector<NodeSharedPtr>,
                        NodeComparator>
        openLists[NUMPROCS];
    std::unordered_map<int, NodeSharedPtr> visited[NUMPROCS];

    // Create buffers to reduce contention
    // Each processor needs N-1 buffers; have 1 dummy buffer to make it N
    NodeBuffer openListsBuffer[NUMPROCS][NUMPROCS];
    for(int i=0; i<NUMPROCS; i++)
        for(int j=0; j<NUMPROCS; j++)
            omp_init_lock(&openListsBuffer[i][j].lock);

    // Create flags
    bool finished[NUMPROCS];
    omp_lock_t finishedLocks[NUMPROCS];
    for(int i=0; i<NUMPROCS; i++)
        omp_init_lock(&finishedLocks[i]);

    // Create root node
    NodeSharedPtr root = std::make_shared<Node>();
    root->f = root->g = root->h = 0.0f;
    root->t = 0;
    root->pos = start;

    // Exit condition
    bool allFinished = false;
    omp_lock_t allFinishedLock;
    omp_init_lock(&allFinishedLock);

    // Push to an open list
    openLists[computeDestination(root->pos)].push(root);

    // Possibly suboptimal path found; -1 if none found yet
    float foundPathG = -1;
    int foundPathT = -1;

    // Logging
    std::vector<BlockLog> blockLogs[NUMPROCS];
    std::chrono::time_point<std::chrono::high_resolution_clock> blockStart[NUMPROCS], blockEnd[NUMPROCS], startTime;
    std::chrono::duration<double, std::micro> blockDur1[NUMPROCS], blockDur2[NUMPROCS];
    const double minTime = 1; // Need to prevent logging of every tiny block; pushes too many things to vector
    int numNodes[NUMPROCS];
    for(int i=0; i<NUMPROCS; i++)
        numNodes[i] = 0;
    startTime = std::chrono::high_resolution_clock::now();
    while(true)
    {
        #pragma omp parallel for
        for(int pid=0; pid<NUMPROCS; pid++)
        {
            while(true)
            {
                // Exit loop and synchronize if all finished, and a path has been found
                blockStart[pid] = std::chrono::high_resolution_clock::now();
                omp_set_lock(&allFinishedLock);
                blockEnd[pid] = std::chrono::high_resolution_clock::now();
                blockDur1[pid] = blockStart[pid] - startTime;
                blockDur2[pid] = blockEnd[pid] - startTime;
                if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                    blockLogs[pid].push_back({pid, 0, blockDur1[pid].count(), blockDur2[pid].count()});
                if(allFinished && foundPathG > -0.01)
                {
                    omp_unset_lock(&allFinishedLock);
                    break;
                }
                omp_unset_lock(&allFinishedLock);

                // Update status
                // Flush buffer and update open lists
                for(int srcID=0; srcID<NUMPROCS; srcID++)
                {
                    // Grab nodes from srcID, load into dstID
                    if(pid==srcID)
                    {
                        // Ignore dummy buffer
                        continue;
                    }
                    NodeBuffer &curBuffer = openListsBuffer[pid][srcID];
                    // Claim, copy, clear, release buffer
                    blockStart[pid] = std::chrono::high_resolution_clock::now();
                    omp_set_lock(&curBuffer.lock);
                    blockEnd[pid] = std::chrono::high_resolution_clock::now();
                    blockDur1[pid] = blockStart[pid] - startTime;
                    blockDur2[pid] = blockEnd[pid] - startTime;
                    if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                        blockLogs[pid].push_back({pid, 0, blockDur1[pid].count(), blockDur2[pid].count()});
                    std::vector<NodeSharedPtr> curNodes = curBuffer.buffer;
                    curBuffer.buffer.clear();
                    omp_unset_lock(&curBuffer.lock);

                    // Push to open list
                    for(const auto& node : curNodes)
                        openLists[pid].push(node);
                }
                // Update finished status
                blockStart[pid] = std::chrono::high_resolution_clock::now();
                omp_set_lock(&finishedLocks[pid]);
                blockEnd[pid] = std::chrono::high_resolution_clock::now();
                blockDur1[pid] = blockStart[pid] - startTime;
                blockDur2[pid] = blockEnd[pid] - startTime;
                if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                    blockLogs[pid].push_back({pid, 0, blockDur1[pid].count(), blockDur2[pid].count()});
                finished[pid] = openLists[pid].empty();
                omp_unset_lock(&finishedLocks[pid]);

                // Check if all complete
                if(openLists[pid].empty())
                {
                    bool localAllFinished = true;
                    for(int pid=0; pid<NUMPROCS; pid++)
                    {
                        blockStart[pid] = std::chrono::high_resolution_clock::now();
                        omp_set_lock(&finishedLocks[pid]);
                        blockEnd[pid] = std::chrono::high_resolution_clock::now();
                        blockDur1[pid] = blockStart[pid] - startTime;
                        blockDur2[pid] = blockEnd[pid] - startTime;
                        if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                            blockLogs[pid].push_back({pid, 0, blockDur1[pid].count(), blockDur2[pid].count()});
                        localAllFinished &= finished[pid];
                        omp_unset_lock(&finishedLocks[pid]);
                        if(!localAllFinished)
                            break;
                    }
                    // Update all finished
                    blockStart[pid] = std::chrono::high_resolution_clock::now();
                    omp_set_lock(&allFinishedLock);
                    blockEnd[pid] = std::chrono::high_resolution_clock::now();
                    blockDur1[pid] = blockStart[pid] - startTime;
                    blockDur2[pid] = blockEnd[pid] - startTime;
                    if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                        blockLogs[pid].push_back({pid, 0, blockDur1[pid].count(), blockDur2[pid].count()});
                    allFinished |= localAllFinished;
                    omp_unset_lock(&allFinishedLock);
                    continue;
                }

                // Start evaluation; grab current node
                NodeSharedPtr cur = openLists[pid].top();
                openLists[pid].pop();
                numNodes[pid]++;

                // If top of priority queue is worse than ceiling, than everything in open list is worse
                // Clear open list
                if(foundPathG > -0.01 && cur->f >= foundPathG)
                {
                    while(!openLists[pid].empty())
                        openLists[pid].pop();
                    finished[pid] = true;
                    continue;
                }

                // Update/put current node in visisted
                int hash = computeHash(cur->pos, cur->t);
                bool makeChildren = true;
                std::unordered_map<int, NodeSharedPtr>::iterator target = visited[pid].find(hash);
                if (target != visited[pid].end())
                {
                    NodeSharedPtr existing_node = target->second;
                    // Claim, compare, update (if better), release node
                    if (cur->g < existing_node->g)
                    {
                        existing_node->g = cur->g;
                        existing_node->f = cur->f;
                        existing_node->t = cur->t;
                        existing_node->parent = cur->parent;
                    }
                    else
                        makeChildren = false;
                }
                else
                {
                    // Node doesn't exist so just add it
                    visited[pid].insert({hash, cur});
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
                for (int dir = 0; makeChildren && dir < NBR_CONNECTEDNESS; dir++)
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

                    // Create child
                    NodeSharedPtr nbr_node = std::make_shared<Node>();
                    nbr_node->pos = nbr_pos;
                    nbr_node->g = cur->g + _travel_cost[dir];
                    nbr_node->h = _heuristicMap[agent_id][nbr_pos.x][nbr_pos.y];
                    nbr_node->f = nbr_node->g + nbr_node->h;
                    nbr_node->t = cur->t + 1;
                    nbr_node->parent = cur;

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
                        NodeBuffer &curBuffer = openListsBuffer[destination][pid];
                        blockStart[pid] = std::chrono::high_resolution_clock::now();
                        omp_set_lock(&curBuffer.lock);
                        blockEnd[pid] = std::chrono::high_resolution_clock::now();
                        blockDur1[pid] = blockStart[pid] - startTime;
                        blockDur2[pid] = blockEnd[pid] - startTime;
                        if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                            blockLogs[pid].push_back({pid, 0, blockDur1[pid].count(), blockDur2[pid].count()});
                        curBuffer.buffer.push_back(nbr_node);
                        omp_unset_lock(&curBuffer.lock);
                    }
                }
            }
            // Synchronization
            blockStart[pid] = std::chrono::high_resolution_clock::now();
        }
        blockEnd[0] = std::chrono::high_resolution_clock::now();
        for(int pid=0; pid<NUMPROCS; pid++)
        {
            blockDur1[pid] = blockStart[pid] - startTime;
            blockDur2[pid] = blockEnd[0] - startTime;
            blockLogs[pid].push_back({pid, 1, blockDur1[pid].count(), blockDur2[pid].count()});
        }
        // Verify open lists and buffers are all empty
        #pragma omp parallel for
        for(int pid=0; pid<NUMPROCS; pid++)
        {
            if(!openLists[pid].empty())
            {
                blockStart[pid] = std::chrono::high_resolution_clock::now();
                omp_set_lock(&allFinishedLock);
                blockEnd[pid] = std::chrono::high_resolution_clock::now();
                blockDur1[pid] = blockStart[pid] - startTime;
                blockDur2[pid] = blockEnd[pid] - startTime;
                if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                    blockLogs[pid].push_back({pid, 2, blockDur1[pid].count(), blockDur2[pid].count()});
                allFinished = false;
                omp_unset_lock(&allFinishedLock);
            }
            else
            {
                for(int srcPID=0; srcPID<NUMPROCS; srcPID++)
                {
                    if(pid==srcPID)
                        continue;
                    NodeBuffer &curBuffer = openListsBuffer[pid][srcPID];
                    if(!curBuffer.buffer.empty())
                    {
                        blockStart[pid] = std::chrono::high_resolution_clock::now();
                        omp_set_lock(&allFinishedLock);
                        blockEnd[pid] = std::chrono::high_resolution_clock::now();
                        blockDur1[pid] = blockStart[pid] - startTime;
                        blockDur2[pid] = blockEnd[pid] - startTime;
                        if(blockDur2[pid].count()-blockDur1[pid].count() > minTime)
                            blockLogs[pid].push_back({pid, 2, blockDur1[pid].count(), blockDur2[pid].count()});
                        allFinished = false;
                        omp_unset_lock(&allFinishedLock);
                        break;
                    }
                }
            }
            // Synchronization
            blockStart[pid] = std::chrono::high_resolution_clock::now();
        }
        blockEnd[0] = std::chrono::high_resolution_clock::now();
        for(int pid=0; pid<NUMPROCS; pid++)
        {
            blockDur1[pid] = blockStart[pid] - startTime;
            blockDur2[pid] = blockEnd[0] - startTime;
            blockLogs[pid].push_back({pid, 3, blockDur1[pid].count(), blockDur2[pid].count()});
        }

        if(allFinished)
        {
            // Display logging
            blockDur1[0] = std::chrono::high_resolution_clock::now() - startTime;
            double sumBlock = 0;
            printf("NUMPROCS = %d\n", NUMPROCS);
            printf("data = [\n");
            for(int pid=0; pid< NUMPROCS; pid++)
            {
                if(blockLogs[pid].empty())
                {
                    continue;
                }
                printf("\t");
                for(const auto& log : blockLogs[pid])
                {
                    printf("[%d,%d,%f,%f], ", log.pid, log.tag, log.tStart, log.tEnd);
                    sumBlock += log.tEnd-log.tStart;
                }
                printf("\n");
            }
            printf("]\n");
            printf("# Total, block, %% = %f, %f, %f %%\n", blockDur1[0].count(), sumBlock, 100*sumBlock/blockDur1[0].count());
            int numNodesTotal = 0;
            for(int i=0; i<NUMPROCS; i++)
            {
                numNodesTotal += numNodes[i];
            }
            printf("# Num nodes = %d; ", numNodesTotal);
            for(int i=0; i<NUMPROCS; i++)
            {
                printf("%d (%2.1f %%), ", numNodes[i], 100.0*numNodes[i]/numNodesTotal);
            }
            printf("\n");
            printf("TMAX = int(%.3f)\n", blockDur1[0].count());
            printf("\n\n");

            // Return
            int hash = computeHash(goal, foundPathT);
            int goalPid = computeDestination(goal);
            if (visited[goalPid].find(hash) != visited[goalPid].end())
            {
                computePath(visited[goalPid][hash], outputPath);
                return true;
            }
            else
            {
                // No path to goal found
                return false;
            }
        }
    }
}

// Return which processor should get this node
int HDAStar::computeDestination(const Point2& pos)
{
    // Stupid simple for now; refine later
    int nodeNum = pos.x*_problem.cols + pos.y;
    return nodeNum % NUMPROCS;
}
