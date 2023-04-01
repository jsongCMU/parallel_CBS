#ifndef MAP_H
#define MAP_H

#include <vector>

struct Point2 {
    int x;
    int y;
};

struct MAPFInstance
{
    int rows;
    int cols;
    int numAgents;

    std::vector<std::vector<bool>> map;

    std::vector<Point2> startLocs;
    std::vector<Point2> goalLocs;
};

#endif