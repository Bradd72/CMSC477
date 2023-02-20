import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time

class Grid:
    # |->x [cols]
    # - y [rows]
    # |
    # v

    def __init__(self,maze_csv):
        self.start_pos = np.argwhere(maze_csv == 2)[0]
        self.goal_pos = np.argwhere(maze_csv == 3)[0]
        self.rows, self.cols = maze_csv.shape
        self.vertices = [[Vertex(i,j) for j in range(self.cols)] for i in range(self.rows)]
        for j in range(self.cols):
            for i in range(self.rows):
                if maze_csv[i][j] == 1:
                    self.add_wall((i,j))
        return

    def add_wall(self,wall_pos):
        i,j = wall_pos
        self.vertices[i][j].adjMatrix = np.zeros((3,3))
        self.vertices[i][j].wall_bool = True
        for dy in range(-1,2):
            if((i+dy)<0 or (i+dy)>=self.rows):   
                continue
            for dx in range(-1,2):
                if((j+dx)<0 or (j+dx)>=self.cols or (dy==0 and dx==0)):
                    continue
                # print("{} {} {} {}".format(i+dy,j+dx,1-dy,1-dx))
                self.vertices[i+dy][j+dx].adjMatrix[1-dy][1-dx] = 0

    def plot(self):
        for j in range(self.cols):
            for i in range(self.rows):
                if self.vertices[i][j].wall_bool:
                    plt.scatter(j,self.rows-i,c="k",marker="x")
        plt.show()

class Vertex:
    def __init__(self,i,j):
        self.adjMatrix = np.ones((3,3))
        self.wall_bool = False
        self.pos = [i, j]
        # [
        #     [o o o]
        #     [o X o]
        #     [o o o]
        # ]
        
    
        
    def __repr__(self):
        return str(self.neighbors)

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.pos= position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
           
                        
def A_Star(map):
    
    path = []
    if not isinstance(map,Grid):
        print("Error not a Grid")
        return path

    open_list = []
    closed_list = []
    start_node = Node(None, map.vertices[map.start_pos].pos)
    open_list.append(start_node)
    while len(open_list) > 0:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node.pos == map.goal_pos:
            path = []
            current = current_node
            while current is not None:
                path.append(current.pos)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (map.cols - 1) or node_position[0] < 0 or node_position[1] > (map.cols -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

    
    return path

if __name__ == '__main__':
    maze_csv = pd.read_csv("C:/Users/jmire/Documents/VS_Code_Projects/CMSC477/CMSC477/Lab_1/Lab1Map.csv",header=None).to_numpy()
    # print(maze_csv)
    maze = Grid(maze_csv)
    maze.plot()