#ifndef SOLVER_UTILS_H
#define SOLVER_UTILS_H

#include <utility>
#include <vector>
#include "MAPFInstance.hpp"

struct Collision
{
    int agent1;
    int agent2;
    int t;
    bool isVertexCollision;

    //If vertex collision then both locations are the same
    std::pair<Point2, Point2> location;
};

inline Collision createVertexCollision(int agent1, int agent2, int t, Point2 loc)
{
    return Collision{agent1, agent2, t, true, std::make_pair(loc, loc)};
}

inline Collision createEdgeCollision(int agent1, int agent2, int t, Point2 locA, Point2 locB)
{
    return Collision{agent1, agent2, t, false, std::make_pair(locA, locB)};
}

struct Constraint
{
    int agentNum;
    int t;
    bool isVertexConstraint;

    //If vertex collision then both locations are the same
    std::pair<Point2, Point2> location;
};

#endif