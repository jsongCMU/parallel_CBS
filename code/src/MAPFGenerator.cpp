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

    std::unordered_set<int> startLocs;
    std::unordered_set<int> goalLocs;

    for(int i=0; i<problem.numAgents; i++)
    {
        Point2 start = getFreeCell(problem, startLocs);
        Point2 goal = getFreeCell(problem, goalLocs);


        problem.startLocs.push_back(start);
        problem.goalLocs.push_back(goal);

        startLocs.insert(computeHash(start.x, start.y, problem.cols));
        goalLocs.insert(computeHash(goal.x, goal.y, problem.cols));
    }
}

Point2 MAPFGenerator::getFreeCell(const MAPFInstance& problem, const std::unordered_set<int> &prevLocs)
{
    int x,y;
    do{
        x = rand() % problem.rows;
        y = rand() % problem.cols;
    } while(problem.map[x][y] || prevLocs.count(computeHash(x, y, problem.cols)));
    return {x,y};
}

int inline MAPFGenerator::computeHash(int x, int y, int cols)
{
    return x * cols + y;
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
