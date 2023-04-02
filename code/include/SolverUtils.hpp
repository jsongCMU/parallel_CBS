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
    Point2 locationA;
    Point2 locationB;
    int t;

    // If is a vertex constraint locA == locB
    bool isVertexConstraint;
};

inline Constraint createVertexConstraint(int agent, const Point2& loc, int t)
{
    return Constraint{agent, loc, loc, t, true};
}

inline Constraint createEdgeConstraint(int agent, const std::pair<Point2, Point2>& locPair, int t)
{
    return Constraint{agent, locPair.first, locPair.second, t, false};
}

#endif