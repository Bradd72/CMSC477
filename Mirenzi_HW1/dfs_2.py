import pandas
import matplotlib.pyplot as plt
import numpy as np
import sys

DEPTH_MAX = 50000
sys.setrecursionlimit(DEPTH_MAX)
goal_position = (0,0)

def dfs(maze,path):
    map = np.zeros(maze.shape)
    k=1
    for step in range(1,5000):
        k = dfs_step(maze,path,map,k)
        if path[goal_position[0],goal_position[1]] != 0:
            break
    return path

def dfs_step(maze,path,map,k):
    rows, cols = maze.shape
    for i in range(rows):
        for j in range(cols):
            if np.ma.masked_array(path,map)[i][j] == k:
                for dx in range(-1,2):
                    if((i+dx)<0 or (i+dx)>rows-1):
                        continue
                    for dy in range(-1,2):
                        if((j+dy)<0 or (j+dy)>cols-1 or (dy==0 and dx==0)):
                            continue
                        if path[i +dx][j+dy] == 0 and maze[i +dx][j+dy] != 1:
                            path[i +dx][j+dy] = k+1
                            return k+1
                map[i][j] = 1
                return k-1

               

if __name__ == '__main__':
    maze = pandas.read_csv("C:/Users/jmire/Documents/VS_Code_Projects/CMSC477/CMSC477/Mirenzi_HW1/Map2.csv").to_numpy()
    path = np.zeros(maze.shape)
    fig, ax = plt.subplots(1,2)
    ax[0].pcolormesh(maze)
    ax[0].set_title("Maze")

    start_position = np.argwhere(maze == 2)[0]
    maze[start_position[0],start_position[1]] = 7
    goal_position = np.argwhere(maze == 3)[0]
    maze[goal_position[0],goal_position[1]] = 8
    
    path[start_position[0],start_position[1]] = 1
    dfs_path = dfs(maze,path)

    
    ax[1].pcolormesh(dfs_path)
    ax[1].set_title("Finding the Path")
    plt.show()
    
