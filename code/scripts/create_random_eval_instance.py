#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from visualize import Animation
import numpy as np

def import_map(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    f.close()
    return my_map

def get_free_location(map, locations):
    x, y = np.random.randint(0, len(map), size=2)

    # Keep regenerating until a free point is found
    while map[x][y] or (x,y) in locations: 
        x, y = np.random.randint(0, len(map), size=2)

    return x,y


def create_random_start_goal(map, num_agents):

    locations = set()

    starts = []
    goals = []

    # Generate unique start locations
    for _ in range(num_agents):
        x, y = get_free_location(map, locations)
        locations.add((x,y))

        starts.append((x, y))

    
    # Generate unique start locations
    for _ in range(num_agents):
        x, y = get_free_location(map, locations)
        locations.add((x,y))

        goals.append((x, y))

    return starts, goals

def save_new_mapf_instance(filename, map, starts, goals):
    f = Path(filename)
    
    f = open(filename, 'w')
    rows = len(map)
    cols = len(map[0])

    f.write(f'{rows} {cols}\n')

    for r in range(rows):
        cur_row = ""
        for c in range(cols):
            if map[r][c]:
                cur_row += "@"
            else:
                cur_row += "."
        
        cur_row += "\n"
        f.write(cur_row)

    f.write(f'{len(starts)}\n')

    for start_loc, goal_loc in zip(starts, goals):
        f.write(f'{start_loc[0]} {start_loc[1]} {goal_loc[0]} {goal_loc[1]}\n')

    f.close()


if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='../instances/evaluation/evaluation_map.txt')
    parser.add_argument('--num_agents', type=int, default=20)
    parser.add_argument('--num_instances', type=int, default=1)
    args = parser.parse_args()

    print("*** Load map ***")
    map = import_map(args.map)


    for i in range(args.num_instances):
        print("*** Create random start + goal locations ***")
        starts, goals = create_random_start_goal(map, args.num_agents)

        save_new_mapf_instance(f'../instances/evaluation/eval{i}.txt', map, starts, goals)