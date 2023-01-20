import os
import random
import numpy as np

if __name__ == '__main__':
    height = int(input('insert map\'s height: '))
    width = int(input('insert map\'s width: '))
    map = np.array([['.' for i in range(width)] for j in range(height)])
    str_map = f'{height} {width}\n'
    num_of_agents = int(input('insert number of agents: '))
    num_of_obstacles = int(input('insert number of obstacles (smaller the height * width + number of agents): '))
    for obstacle in range(num_of_obstacles):
        while True:
            x = random.randint(0, width-1)
            y = random.randint(0, height-1)
            if map[x][y] == '.':
                map[x][y] = '@'
                break
    for row in map:
        str_map += (' '.join(row) + '\n')
    agents_list = []
    for agent in range(num_of_agents):
        while True:
            x = random.randint(0, width-1)
            y = random.randint(0, height-1)
            if map[x][y] == '.':
                start = (x, y)
                map[x][y] = '@'
                break
        while True:
            x = random.randint(0, width-1)
            y = random.randint(0, height-1)
            if map[x][y] == '.':
                goal = (x, y)
                map[x][y] = '@'
                break
        agents_list.append([str(start[0]), str(start[1]), str(goal[0]), str(goal[1])])
    str_map += (str(num_of_agents) + '\n')
    for agent in agents_list:
        str_map += (' '.join(agent) + '\n')
    filename = input('filename: ')
    if filename[-4] != '.':
        filename = f'{filename}.txt'
    f = open(filename, "w")
    f.write(str_map)
    f.close()
