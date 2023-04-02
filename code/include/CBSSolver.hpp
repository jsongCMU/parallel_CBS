#ifndef CBS_SOLVER_H
#define CBS_SOLVER_H

#include "MAPFInstance.hpp"
#include <memory>
#include <tuple>

class CBSSolver
{
public:
    CBSSolver();

    std::vector<std::vector<Point2>> solve(MAPFInstance instance);

private:
    struct Collision
    {
        int agent1;
        int agent2;
        int t;
        bool isVertexCollision;
        std::tuple<Point2,Point2> location;
    };

    struct Constraint
    {
        int agentNum;
        Point2 location;
        int t;
    };

    struct CTNode
    {
        float cost;
        std::vector<std::vector<Point2>> paths;
        std::vector<Collision> collisionList;
        std::vector<Constraint> constraintList;
        int id;
    };

    typedef std::shared_ptr<CTNode> CTNodeSharedPtr;

    class CTNodeComparator
    {
    public:
        bool operator()(const CTNodeSharedPtr &a, const CTNodeSharedPtr &b) const
        {
            if (a->cost == b->cost)
            {
                if (a->collisionList.size() == b->collisionList.size())
                {
                    return a->id > b->id;
                }

                return a->collisionList.size() > b->collisionList.size();
            }

            return a->cost > b->cost;
        }
    };

    int numNodesGenerated;

};

#endif