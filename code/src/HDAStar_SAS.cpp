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

// Return which processor should get this node
int HDAStar::computeDistribution(const Point2& pos)
{
    // Stupid simple for now; refine later
    int nodeNum = pos.x*_problem.cols + pos.y;
    return nodeNum % NUMPROCS;
}