#include "MAPFLoader.hpp"
#include <iostream>
#include <fstream>
#include <bits/stdc++.h>
#include <algorithm>

MAPFInstance MAPFLoader::loadInstanceFromFile(const std::string& fileName)
{
    std::fstream txtFile;
    txtFile.open(fileName, std::ios::in);

    MAPFInstance result;

    if (txtFile.is_open()) {
        std::stringstream ss;
        std::string contents;

        ss << txtFile.rdbuf();
        contents = ss.str();
        
        parseText(contents, result);
    }

    // Debugging code
    // printf("%d %d\n", result.rows, result.cols);
    
    // for (int r = 0; r < result.rows; r++)
    // {
    //     for (int c = 0; c < result.cols; c++)
    //     {
    //         if (result.map[r][c])
    //             printf("@ ");
    //         else
    //             printf(". ");
    //     }
    //     printf("\n");
    // }

    // printf("%d\n", result.numAgents);

    // for (int i = 0; i < result.numAgents; i++)
    // {
    //     printf("%d %d %d %d\n", result.startLocs[i].x, result.startLocs[i].y, result.goalLocs[i].x, result.goalLocs[i].y);
    // }

    return result;
}

void MAPFLoader::parseText(std::string text, MAPFInstance &result)
{
    int end_first_line = text.find("\n");
    parseRowsAndCols(text.substr(0, end_first_line), result);
    text.erase(text.begin(), text.begin() + end_first_line + 1);

    std::string digits = "0123456789";
    int map_len = text.find_first_of(digits);
    parseMap(text.substr(0, map_len), result);
    text.erase(text.begin(), text.begin() + map_len);

    parseAgentDetails(text, result);
}

void MAPFLoader::parseRowsAndCols(std::string line, MAPFInstance &result)
{
    std::string value;
    std::stringstream ss(line);
    bool first = true;

    while(ss >> value)
    {
        if (first)
        {
            result.rows = std::stoi(value);
            first = false;
        }
        else
        {
            result.cols = std::stoi(value);
            break;
        }
    }
}

void MAPFLoader::parseMap(std::string map_as_txt, MAPFInstance &result)
{
    // Remove any whitespace from the map   
    map_as_txt.erase(std::remove(map_as_txt.begin(), map_as_txt.end(), ' '), map_as_txt.end());
    map_as_txt.erase(std::remove(map_as_txt.begin(), map_as_txt.end(), '\n'), map_as_txt.end());

    result.map.resize(result.rows);

    int cur_pos = 0;
    for (int r = 0; r < result.rows; r++)
    {
        result.map[r].resize(result.cols);

        for (int c = 0; c < result.cols; c++)
        {
            if (map_as_txt[cur_pos] == '@')
                result.map[r][c] = true;
            else if (map_as_txt[cur_pos] == '.')
                result.map[r][c] = false;

            cur_pos++;
        }
        cur_pos++;
    }
}


void MAPFLoader::parseAgentDetails(std::string agent_info, MAPFInstance &result)
{
    int end_first_line = agent_info.find("\n");
    result.numAgents = std::stoi(agent_info.substr(0, end_first_line));
    
    agent_info.erase(agent_info.begin(), agent_info.begin() + end_first_line + 1);

    result.startLocs.reserve(result.numAgents);
    result.goalLocs.reserve(result.numAgents);
    

    int values[4];
    for (int i = 0; i < result.numAgents; i++)
    {
        // std::cout << "Agent Info = " << agent_info << std::endl;
        // std::cout << "-------" << std::endl;
        int end_line = agent_info.find("\n");
        std::string line = agent_info.substr(0, end_line);
        for (int j = 0; j < 3; j++)
        {
            int end_next_int = line.find(" ");
            values[j] = std::stoi(line.substr(0, end_next_int));
            line.erase(line.begin(), line.begin() + end_next_int + 1);
        }
        values[3] = std::stoi(line.substr(0, line.length()));

        result.startLocs.push_back(Point2{values[0], values[1]});
        result.goalLocs.push_back(Point2{values[2], values[3]});
        
        agent_info.erase(agent_info.begin(), agent_info.begin() + end_line + 1);
    }
}
