

"""
    0: open
    1: wall

"""

WALL_VALUE = 1
ORIGIN_VALUE = 2

import numpy as np

def meter_to_scale(meters_value):
    multiplier = 10
    return int(meters_value*multiplier)

if __name__ == "__main__":
    maze_width_meters = 6
    maze_height_meters = 3
    leftEdge_BlockZone_from_leftEdgeMaze_meters = 4
    width_BlockZone_meters = 1
    height_BlockZone_meters = 2

    maze = np.zeros((meter_to_scale(maze_height_meters),meter_to_scale(maze_width_meters)))
    (maze_height, maze_width) = np.shape(maze)
    river_center = (int(maze_height/2),int(maze_width/2))
    maze[:,0]=WALL_VALUE;maze[:,maze_width-1]=WALL_VALUE
    maze[0,:]=WALL_VALUE;maze[maze_height-1,:]=WALL_VALUE
    maze[:,river_center[1]]=WALL_VALUE
    leftEdge_BlockZone_from_leftEdgeMaze = meter_to_scale(leftEdge_BlockZone_from_leftEdgeMaze_meters)
    width_BlockZone = meter_to_scale(width_BlockZone_meters)
    height_BlockZone = meter_to_scale(height_BlockZone_meters)
    

    for i in range(leftEdge_BlockZone_from_leftEdgeMaze,leftEdge_BlockZone_from_leftEdgeMaze+width_BlockZone):
        for j in range(int(river_center[0]+height_BlockZone/(-2)),int(river_center[0]+height_BlockZone/(2))):
            maze[j,i] = WALL_VALUE


    np.savetxt('Final_Lab/Final_Lab_maze.csv', maze,delimiter=",")