#include "../include/MAPFLoader.hpp"

#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <bits/stdc++.h>

std::string instanceToString(MAPFInstance instance)
{
    std::string result;
    result += std::to_string(instance.rows);
    result += " ";
    result += std::to_string(instance.cols);

    result += "\n";
    
    for (int r = 0; r < instance.rows; r++)
    {
        for (int c = 0; c < instance.cols; c++)
        {
            if (instance.map[r][c])
                result += "@";
            else
                result += ".";

            if (c != instance.cols - 1)
                result += " ";
                
        }
        result += "\n";
    }

    result += std::to_string(instance.numAgents);
    result += "\n";

    for (int i = 0; i < instance.numAgents; i++)
    {
        result += std::to_string(instance.startLocs[i].x);
        result += " ";
        result += std::to_string(instance.startLocs[i].y);
        result += " ";
        result += std::to_string(instance.goalLocs[i].x);
        result += " ";
        result += std::to_string(instance.goalLocs[i].y);

        if (i < instance.numAgents - 1)
            result += "\n";
    }

    return result;
}

int main()
{
    std::string instanceDir = "../instances/";
    MAPFLoader loader;

    for (const auto & filename : std::filesystem::directory_iterator(instanceDir))
    {
        // Convert mapf instance to string
        MAPFInstance mapfProblem = loader.loadInstanceFromFile(filename.path());
        std::string loadedData = instanceToString(mapfProblem);
        
        // Load file into string
        std::fstream txtFile;
        txtFile.open(filename.path(), std::ios::in);

        if (txtFile.is_open()) {
            std::stringstream ss;
            std::string contents;

            ss << txtFile.rdbuf();
            contents = ss.str();

            int lastIdx = contents.find_last_of("0123456789");
            contents = contents.substr(0, lastIdx);
        
            assert(contents.compare(loadedData));
        }
    }
}