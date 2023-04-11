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

struct LockedNode : Node
{
    omp_lock_t lock;
};

typedef std::shared_ptr<LockedNode> LNodeSharedPtr;

struct LNodeBuffer
{
    omp_lock_t lock;
    std::vector<LNodeSharedPtr> buffer;
};

class HDAStar : public AStar
{
public:
    HDAStar(const MAPFInstance& mapfInstance);
private:
    int computeDistribution(const Point2& pos);
};

#endif