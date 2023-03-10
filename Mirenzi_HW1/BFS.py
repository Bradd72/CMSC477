import pandas
import matplotlib.pyplot as plt
import numpy as np
import sys
from matplotlib.animation import PillowWriter

# metadata = dict(title="Movie", artist="codinglikemad")
writer = PillowWriter(fps=15)
goal_position = (0,0)

def bfs(maze,path):
    with writer.saving(figure,"BFS.gif",100):
        for step in range(1,20000):
            if not step % 5: ## the grabing frames slows this down a ton
                ax[1].pcolormesh(path,rasterized=True)
                writer.grab_frame()
            bfs_step(maze,path,step)
            if path[goal_position[0],goal_position[1]] != 0:
                break
        i, j = goal_position
        k = path[i][j]
        shortest_path = maze
        while k > 1:
            ax[2].pcolormesh(shortest_path,rasterized=True)
            writer.grab_frame()
            for dx in range(-1,2):
                for dy in range(-1,2):
                    if path[i +dx][j+dy] == k-1:
                        i+=dx
                        j+=dy
                        shortest_path[i,j] = 6
                        k-=1
    return path, shortest_path

def bfs_step(maze,path,k):
    rows, cols = maze.shape
    for i in range(rows):
        for j in range(cols):
            if path[i][j] == k:
                for dx in range(-1,2):
                    for dy in range(-1,2):
                        if path[i +dx][j+dy] == 0 and maze[i +dx][j+dy] != 1:
                            path[i +dx][j+dy] = k+1
               
figure, ax = plt.subplots(1,3)

if __name__ == '__main__':
    maze = pandas.read_csv("C:/Users/jmire/Documents/VS_Code_Projects/CMSC477/CMSC477/Mirenzi_HW1/Map3.csv").to_numpy()
    path = np.zeros(maze.shape)
    ax[0].pcolormesh(maze)
    ax[0].set_title("Maze")
    ax[1].set_title("Finding the Path")
    ax[2].set_title("Shortest Path")
    start_position = np.argwhere(maze == 2)[0]
    maze[start_position[0],start_position[1]] = 7
    goal_position = np.argwhere(maze == 3)[0]
    maze[goal_position[0],goal_position[1]] = 8
    path[start_position[0],start_position[1]] = 1
    bfs_path,shortest_path = bfs(maze,path)

    
