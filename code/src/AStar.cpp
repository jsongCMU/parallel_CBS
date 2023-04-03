#include "AStar.hpp"
#include <queue>
#include <cstdio>
#include <unordered_map>
#include <algorithm>

/* 
Defines connectivity of neighbourhood
 4 or 8 (+1 for wait) works but 8 is always better
 NOTE: If you change to 4, need to change
 heuristic as well.
*/
#define NBR_CONNECTEDNESS 9

ConstraintsTable AStar::buildConstraintsTable(const std::vector<Constraint>& constraints, const int agent_id, int& maxTimestep)
{
    // ConstraintsTable: timestep -> vector of constraints for specific agent
    maxTimestep = 0;
    ConstraintsTable constraintsTable;
    for(const auto& constraint : constraints)
    {
        // Only care about constraints for specified agent
        if(constraint.agentNum != agent_id)
            continue;
        // Add/append to constraints
        const int curTime = constraint.t;
        if (constraintsTable.find(curTime) != constraintsTable.end())
            constraintsTable[curTime].push_back(constraint);
        else
            constraintsTable.insert({curTime, {constraint}});
        // Update max time
        maxTimestep = (curTime > maxTimestep) ? curTime : maxTimestep;
    }
    return constraintsTable;
}

bool AStar::isConstrained(const Point2& currLoc, const Point2& nextLoc, const int nextTime, const ConstraintsTable& constraintsTable)
{
    if (constraintsTable.find(nextTime) == constraintsTable.end())
        return false;
    const auto& constraints = constraintsTable.at(nextTime);
    for(const auto& constraint : constraints)
    {
        if(constraint.t != nextTime)
            continue;
        if(constraint.isVertexConstraint)
            if(constraint.location.first == nextLoc)
                return true;
        else 
            if(
                (constraint.location.first == currLoc && constraint.location.second == nextLoc) || 
                (constraint.location.first == nextLoc && constraint.location.second == currLoc))
                return true;
    }
    return false;
}

bool AStar::solve(const MAPFInstance& problem, const int agent_id, const std::vector<Constraint>& constraints)
{
    Point2 start = problem.startLocs[agent_id];
    Point2 goal = problem.goalLocs[agent_id];

    // Create constraints table
    int maxTimestep;
    ConstraintsTable constraintsTable = buildConstraintsTable(constraints, agent_id, maxTimestep);

    // Check validity of start and end
    if(start.x >= problem.rows || start.y >= problem.cols)
    {
        printf("* ERR: AStar Failed: Start position not in bounds\n");
        return false;
    }
    else if(goal.x >= problem.rows || goal.y >= problem.cols)
    {
        printf("* ERR: AStar Failed: Goal position not in bounds\n");
        return false;
    }

    // Create open list and visited map
    std::priority_queue <NodeSharedPtr, 
                         std::vector<NodeSharedPtr>, 
                         NodeComparator 
                        > open_list;

    std::unordered_map<int, NodeSharedPtr> visited;

    NodeSharedPtr root = std::make_shared<Node>();
    root->f = root->g = root->h = 0.0f;
    root->t = 0;
    root->pos = start;

    open_list.push(root);
    visited.insert({computeHash(start, root->t), root});

    while (!open_list.empty())
    {
        NodeSharedPtr cur = open_list.top();
        open_list.pop();

        if (cur->pos == goal)
        {
            computePath(cur);
            return true;
        }

        cur->isClosed = true;

        for (int dir = 0; dir < NBR_CONNECTEDNESS; dir++)
        {
            Point2 nbr_pos = Point2{cur->pos.x + _dx[dir], cur->pos.y + _dy[dir]};

            // Skip if out of bounds
            if (nbr_pos.x >= problem.rows || nbr_pos.y >= problem.cols || nbr_pos.x <0 || nbr_pos.y < 0)
                continue;

            // Skip if inside obstacle
            if (problem.map[nbr_pos.x][nbr_pos.y])
                continue;
            
            // Skip if violates constraints table
            if (isConstrained(cur->pos, nbr_pos, cur->t+1, constraintsTable))
                continue;

            int hash = computeHash(nbr_pos, cur->t+1);

            // Check if a node already exists
            if (visited.find(hash) != visited.end())
            {
                NodeSharedPtr existing_node = visited[hash];

                // If node is in closed list we cant do better, so skip
                if (existing_node->isClosed)
                    continue;

                float cur_travel_cost = cur->g + _travel_cost[dir];

                // If current path to existing node is shorter, then edit existing node
                if (cur_travel_cost < existing_node->g)
                {
                    existing_node->g = cur_travel_cost;
                    existing_node->f = existing_node->h + cur_travel_cost;
                    existing_node->parent = cur;
                    existing_node->t = cur->t+1;
                    
                    // We need to update the nodes position in the prio queue but 
                    // either we create a duplicate (and suffer overhead of re-expanding
                    // node) or we heapify the current priority queue (and suffer overhead
                    // of O(N) for elements in priority queue)
                    open_list.push(existing_node);
                }
            }
            else
            {
                // Node doesn't exist so just add it
                NodeSharedPtr nbr_node = std::make_shared<Node>();
                nbr_node->pos = nbr_pos;
                nbr_node->g = cur->g + _travel_cost[dir];
                nbr_node->h = computeHeuristic(nbr_pos, goal);
                nbr_node->f = nbr_node->g + nbr_node->h;
                nbr_node->t = cur->t+1;
                nbr_node->parent = cur;

                // Add to visited
                visited.insert({hash, nbr_node});

                open_list.push(nbr_node);
            }
        }
    }

    return false;
}


void AStar::computePath(NodeSharedPtr goal)
{
    std::vector<Point2> results;
    NodeSharedPtr cur = goal;
    while(cur != nullptr)
    {
        results.push_back(cur->pos);
        cur = cur->parent;
    }

    // Reverse path so we have from start to goal
    std::reverse(results.begin(), results.end());

    _path = results;
}


float AStar::computeHeuristic(const Point2& start, const Point2& goal)
{
    // Use octile distance since its an exact heuristic for 8 connected grids
    int dx = abs(start.x - goal.x);
    int dy = abs(start.y - goal.y);
    return std::max(dx, dy) - std::min(dx, dy) + DIAGONAL * std::min(dx, dy);
}


int AStar::computeHash(const Point2& pos, const int t)
{
    // Only works for maps with a height less than 100000
    return (pos.x * 1000 + pos.y)*1000 + t;
}