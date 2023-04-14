#ifndef HDASTAR_H
#define HDASTAR_H

#include <vector>
#include <memory>
#include <unordered_map>
#include <omp.h>

#include "MAPFInstance.hpp"
#include "SolverUtils.hpp"
#include "AStar.hpp"

#define DIAGONAL (1.41421356)
#define NUMPROCS (2)

struct NodeBuffer
{
    omp_lock_t lock;
    std::vector<NodeSharedPtr> buffer;
};

class HDAStar : public AStar
{
public:
    HDAStar(const MAPFInstance& mapfInstance);
    bool solve(const int agent_id, const std::vector<Constraint> &constraints, std::vector<Point2> &outputPath);
private:
    int computeDestination(const Point2& pos);
};

#endif