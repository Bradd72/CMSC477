import pandas
import matplotlib.pyplot as plt
import numpy as np
import sys

DEPTH_MAX = 50000
sys.setrecursionlimit(DEPTH_MAX)
goal_position = (0,0)

def bfs(maze):
    for step in range(1,500000):
        bfs_step(maze,path,step)
        if path[goal_position[0],goal_position[1]] != 0:
            print("success")
            break
    return path

def bfs_step(maze,path,k):
    rows, cols = maze.shape
    for i in range(rows):
        for j in range(cols):
            if path[i][j] == k:
                for dx in range(-1,2):
                    for dy in range(-1,2):
                        if path[i +dx][j+dy] == 0 and maze[i +dx][j+dy] != 1 and maze[i +dx][j+dy] != 4:
                            path[i +dx][j+dy] = k+1
                            maze[i +dx][j+dy] = 4
                            return
               

if __name__ == '__main__':
    maze = pandas.read_csv("C:/Users/jmire/Documents/VS_Code_Projects/CMSC477/CMSC477/Mirenzi_HW1/Map1.csv").to_numpy()
    path = np.zeros(maze.shape)
    fig, ax = plt.subplots(1,2)
    

    start_position = np.argwhere(maze == 2)[0]
    maze[start_position[0],start_position[1]] = 7
    goal_position = np.argwhere(maze == 3)[0]
    maze[goal_position[0],goal_position[1]] = 8
    
    path[start_position[0],start_position[1]] = 1
    bfs_path = bfs(maze)

    ax[0].pcolormesh(maze)
    ax[0].set_title("Maze")
    ax[1].pcolormesh(bfs_path)
    ax[1].set_title("Finding the Path")
    plt.show()
    
