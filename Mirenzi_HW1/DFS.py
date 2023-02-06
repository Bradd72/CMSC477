import pandas
import matplotlib.pyplot as plt
import numpy as np
import sys

DEPTH_MAX = 5000
Success = False
sys.setrecursionlimit(DEPTH_MAX)

def dfs(maze,posX,posY):
    global Success
    if Success:
        return
    if maze[posX,posY] == 1 or maze[posX,posY] == 4:
        return
    elif maze[posX,posY] == 8:
        Success = True
        print("success!")
        return
    elif maze[posX,posY] == 0:
        maze[posX,posY] = 4
    for dx in range(-1,2):
        for dy in range(-1,2):
            if (dx,dy) == (0,0):
                break
            dfs(maze,posX+dx,posY-dy)
            

        


if __name__ == '__main__':
    df = pandas.read_csv("C:/Users/jmire/Documents/VS_Code_Projects/CMSC477/CMSC477/Mirenzi_HW1/Map1.csv")
    maze = df.to_numpy()
    start_position = np.argwhere(maze == 2)[0]
    maze[start_position[0],start_position[1]] = 7
    goal_position = np.argwhere(maze == 3)[0]
    maze[goal_position[0],goal_position[1]] = 8
    dfs(maze,start_position[0],start_position[1])
    plt.pcolormesh(maze)
    plt.show()

    

