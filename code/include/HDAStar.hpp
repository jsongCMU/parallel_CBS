#ifndef HDASTAR_H
#define HDASTAR_H

#include <vector>
#include <memory>
#include <unordered_map>
#include "MAPFInstance.hpp"
#include "SolverUtils.hpp"
#include "AStar.hpp"

#define DIAGONAL (1.41421356)

class HDAStar : public AStar
{
public:
    HDAStar(const MAPFInstance& mapfInstance);
private:
    int computeDistribution(const Point2& pos);
};

#endif