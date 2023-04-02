#!/usr/bin/python
import argparse
import glob
from pathlib import Path
# from cbs import CBSSolver
# from pbs import PBSSolver
# from independent import IndependentSolver
# from joint_state import JointStateSolver
# from prioritized import PrioritizedPlanningSolver
from visualize import Animation
# from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        if my_map[sx][sy]:
           raise BaseException("Agent {} start postion {} {} in collision".format(a, sx, sy)) 
        if my_map[gx][gy]:
           raise BaseException("Agent {} goal postion {} {} in collision".format(a, gx, gy)) 
        if (sx, sy) in starts:
           raise BaseException("Agent {} start postion {} {} repeated".format(a, sx, sy)) 
        if (gx, gy) in goals:
           raise BaseException("Agent {} goal postion {} {} repeated".format(a, gx, gy)) 
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals

def import_results(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: MAPF file name
    line = f.readline()
    mapf_filename = " ".join(line.split())
    # second line: sum of costs
    line = f.readline()
    soc = " ".join(line.split())
    # Remaining: path for each agent
    paths = []
    line = f.readline()
    while(line != ""):
        # Get list of strings, where each string is a pair of ints
        path=line.replace('(', '').replace(')', '').replace(';', '').split()
        # Conver to list of tuples, each
        path = list(map(lambda loc: tuple(map(int, loc.split(','))), path))
        paths.append(path)
        line = " ".join(f.readline().split())
    # rows, columns = [int(x) for x in line.split(' ')]
    # rows = int(rows)
    # columns = int(columns)
    # # #rows lines with the map
    # my_map = []
    # for r in range(rows):
    #     line = f.readline()
    #     my_map.append([])
    #     for cell in line:
    #         if cell == '@':
    #             my_map[-1].append(True)
    #         elif cell == '.':
    #             my_map[-1].append(False)
    # # #agents
    # line = f.readline()
    # num_agents = int(line)
    # # #agents lines with the start/goal positions
    # starts = []
    # goals = []
    # for a in range(num_agents):
    #     line = f.readline()
    #     sx, sy, gx, gy = [int(x) for x in line.split(' ')]
    #     if my_map[sx][sy]:
    #        raise BaseException("Agent {} start postion {} {} in collision".format(a, sx, sy)) 
    #     if my_map[gx][gy]:
    #        raise BaseException("Agent {} goal postion {} {} in collision".format(a, gx, gy)) 
    #     if (sx, sy) in starts:
    #        raise BaseException("Agent {} start postion {} {} repeated".format(a, sx, sy)) 
    #     if (gx, gy) in goals:
    #        raise BaseException("Agent {} goal postion {} {} repeated".format(a, gx, gy)) 
    #     starts.append((sx, sy))
    #     goals.append((gx, gy))
    f.close()
    return mapf_filename, soc, paths


if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser(description='Create animation using results file')
    parser.add_argument('--file', type=str, default='outputs/result.txt',
                        help='The name of the file to animate')
    args = parser.parse_args()

    # Parse results file
    mapf_file, soc, paths = import_results(args.file)

    print("***Import an instance***")
    my_map, starts, goals = import_mapf_instance(mapf_file)
    print_mapf_instance(my_map, starts, goals)

    print("***Test paths on a simulation***")
    animation = Animation(my_map, starts, goals, paths)
    animation.save("outputs/result.mp4", 1.0)
