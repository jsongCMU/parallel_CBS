#ifndef MAPF_GENERATOR_H
#define MAPF_GENERATOR_H

#include "MAPFInstance.hpp"
#include "MAPFLoader.hpp"
#include "AStar.hpp"

class MAPFGenerator : public MAPFLoader
{
    public:
    MAPFInstance generateProblem(const std::string& fileName, const int numAgents);

    private:
    void parseText(std::string text, MAPFInstance &result);
    void generateStartGoal(MAPFInstance& problem);
    Point2 getFreeCell(const MAPFInstance& problem);
    bool isValid(const MAPFInstance& problem);
};

#endif