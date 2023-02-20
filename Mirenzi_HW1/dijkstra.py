import pandas
import matplotlib.pyplot as plt
import numpy as np
import sys
from matplotlib.animation import PillowWriter

# metadata = dict(title="Movie", artist="codinglikemad")
writer = PillowWriter(fps=15)
fig, ax = plt.subplots(1,3)
goal_position = (0,0)

def dijkstra(maze,path):
    with writer.saving(fig,"dijkstra.gif",100):
        map = np.zeros(maze.shape)
        shortest_path = maze
        for step in range(1,20000):
            if not step % 100: ## the grabing frames slows this down a ton
                ax[1].pcolormesh(path,rasterized=True)
                writer.grab_frame()
            dijkstra_step(maze,path,map,step)
            if path[goal_position[0],goal_position[1]] != maze.shape[0]**2:
                path[path == maze.shape[0]**2] = 0
                ax[1].pcolormesh(path,rasterized=True)
                ax[2].set_title("Shortest Path")
                i, j = goal_position
                k = path[i][j]
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
                break
        
    return path, shortest_path, map


def dijkstra_step(maze,path,map,k):
    rows, cols = maze.shape
    closest_point_distance = np.min(np.ma.masked_array(path,map))
    for i in range(rows):
        for j in range(cols):
            if path[i][j] == closest_point_distance and map[i][j] != 1:
                for dx in range(-1,2):
                    if((i+dx)<0 or (i+dx)>rows):
                        continue
                    for dy in range(-1,2):
                        if((j+dy)<0 or (j+dy)>cols or (dy==0 and dx==0)):
                            continue
                        if maze[i +dx][j+dy] != 1 and path[i +dx][j+dy] > closest_point_distance+1:
                            path[i +dx][j+dy] = closest_point_distance+1
                            
                map[i][j] = 1
                return
    print("Search got stuck")
    return

                
               

if __name__ == '__main__':
    maze = pandas.read_csv("C:/Users/jmire/Documents/VS_Code_Projects/CMSC477/CMSC477/Mirenzi_HW1/Map1.csv").to_numpy()
    path = np.ones(maze.shape)*maze.shape[0]**2
    ax[0].pcolormesh(maze)
    ax[0].set_title("Maze")
    ax[1].set_title("Finding the Path")
    start_position = np.argwhere(maze == 2)[0]
    maze[start_position[0],start_position[1]] = 7
    goal_position = np.argwhere(maze == 3)[0]
    maze[goal_position[0],goal_position[1]] = 8
    path[start_position[0],start_position[1]] = 1
    dji_path,shortest_path,map = dijkstra(maze,path)