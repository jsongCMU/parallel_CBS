#ifndef ASTAR_H
#define ASTAR_H

#include "MAPFInstance.hpp"
#include <vector>
#include <memory>

#define DIAGONAL (1.41421356)

struct Node
{
    Point2 pos;
    float g; // Cost to come
    float h; // Cost to go (heuristic)
    float f; // Total cost (estimate)

    bool isClosed;
    std::shared_ptr<Node> parent;
};

typedef std::shared_ptr<Node> NodeSharedPtr;

class AStar
{
public:
    bool solve(const MAPFInstance& problem, const int agent_id);

    std::vector<Point2> getPath()
    {
        return _path;
    }

private:
    void computePath(NodeSharedPtr goal);
    float computeHeuristic(const Point2& start, const Point2& goal);
    int computeHash(const Point2& pos);
    std::vector<Point2> _path;

    const int _dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int _dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float _travel_cost[8] = {1, 1, 1, 1, DIAGONAL, DIAGONAL, DIAGONAL, DIAGONAL};
};

class NodeComparator
{
public:
    bool operator()(const NodeSharedPtr &a, const NodeSharedPtr &b) const
    {
        return a->f > b->f;
    }
};

#endif