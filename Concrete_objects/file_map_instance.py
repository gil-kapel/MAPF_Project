from pathlib import Path as P
from Abstract_objects import MapfInstance, WayPoint
from Concrete_objects import TimedWayPoint
import numpy as np
from MAPF_exceptions import NoFileException


class FileMapInstance(MapfInstance):
    def create_map(self, file):
        f = P(file)
        if not f.is_file():
            raise NoFileException(file + " does not exist.")
        f = open(file, 'r')
        # first line: #rows #columns
        line = f.readline()
        rows, columns = [int(x) for x in line.split(' ')]
        rows = int(rows)
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
        if file[-3:] == 'map':
            my_map = np.array(my_map).T.tolist()
        # agents
        while True:
            line = f.readline()
            if line != '\n':
                break
        num_agents = int(line)
        # agents lines with the start/goal positions
        starts = []
        goals = []
        for a in range(num_agents):
            line = f.readline()
            sx, sy, gx, gy = [int(x) for x in line.split(' ')]
            starts.append(WayPoint(sx, sy))
            goals.append(TimedWayPoint(gx, gy))
        f.close()
        self.map = my_map
        return starts, goals


