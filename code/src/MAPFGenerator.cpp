#include "MAPFGenerator.hpp"
#include <iostream>
#include <fstream>
#include <bits/stdc++.h>
#include <algorithm>
#include <stdlib.h>

MAPFInstance MAPFGenerator::generateProblem(const std::string& fileName, const int numAgents)
{
    MAPFInstance problem;
    // Read map
    std::fstream txtFile;
    txtFile.open(fileName, std::ios::in);
    if (txtFile.is_open())
    {
        std::stringstream ss;
        std::string contents;

        ss << txtFile.rdbuf();
        contents = ss.str();

        parseText(contents, problem);
    }
    else
    {
        printf("ERROR: File was not opened\n");
    }
    txtFile.close();

    // Set number of agents
    problem.numAgents = numAgents;

    // Set valid start and end goals
    do
    {
        generateStartGoal(problem);
    } while(!isValid(problem));

    return problem;
}

void MAPFGenerator::parseText(std::string text, MAPFInstance &problem)
{
    // Parse text file to get map
    int end_first_line = text.find("\n");
    parseRowsAndCols(text.substr(0, end_first_line), problem);
    text.erase(text.begin(), text.begin() + end_first_line + 1);

    std::string digits = "0123456789";
    int map_len = text.find_first_of(digits);
    parseMap(text.substr(0, map_len), problem);
    text.erase(text.begin(), text.begin() + map_len);
}

void MAPFGenerator::generateStartGoal(MAPFInstance& problem)
{
    // Generate random start and end goals
    problem.startLocs.clear();
    problem.goalLocs.clear();
    for(int i=0; i<problem.numAgents; i++)
    {
        problem.startLocs.push_back(getFreeCell(problem));
        problem.goalLocs.push_back(getFreeCell(problem));
    }
}

Point2 MAPFGenerator::getFreeCell(const MAPFInstance& problem)
{
    int x,y;
    do{
        x = rand() % problem.rows;
        y = rand() % problem.cols;
    } while(problem.map[x][y]);
    return {x,y};
}

bool MAPFGenerator::isValid(const MAPFInstance& problem)
{
    // Check paths exist from start to goal
    AStar lowLevelSolver(problem);
    for (int id = 0; id < problem.numAgents; id++)
    {
        std::vector<Point2> path;
        bool found = lowLevelSolver.solve(id, {}, path);
        if (!found)
            return false;
    }
    return true;
}
