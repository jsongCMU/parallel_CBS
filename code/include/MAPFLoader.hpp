#ifndef MAPF_LOADER_H
#define MAPF_LOADER_H

#include <string>
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

class MAPFLoader
{
    public:

    MAPFInstance loadInstanceFromFile(const std::string& fileName);

    private:
    void parseText(std::string text, MAPFInstance &result);
    void parseRowsAndCols(std::string line, MAPFInstance &result);
    void parseMap(std::string map_as_txt, MAPFInstance &result);
    void parseAgentDetails(std::string agent_info, MAPFInstance &result);
};

#endif