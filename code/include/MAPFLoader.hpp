#ifndef MAPF_LOADER_H
#define MAPF_LOADER_H

#include <string>
#include "MAPFInstance.hpp"

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