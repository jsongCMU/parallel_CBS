#ifndef MAPF_GENERATOR_H
#define MAPF_GENERATOR_H

#include "MAPFInstance.hpp"
#include "MAPFLoader.hpp"
#include "AStar.hpp"
#include <unordered_set>

class MAPFGenerator : public MAPFLoader
{
public:
    MAPFInstance generateProblem(const std::string &fileName, const int numAgents);

private:
    void parseText(std::string text, MAPFInstance &result);
    void generateStartGoal(MAPFInstance &problem);
    Point2 getFreeCell(const MAPFInstance &problem, const std::unordered_set<int> &prevLocs);
    int inline computeHash(int x, int y, int cols);
    bool isValid(const MAPFInstance &problem);
};

#endif