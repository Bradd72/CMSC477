

"""
    0: open
    1: wall

"""

WALL_VALUE = 1
ORIGIN_VALUE = 2

import numpy as np

def meter_to_scale(meters_value):
    multiplier = 15
    return int(meters_value*multiplier)

if __name__ == "__main__":
    maze_width_meters = 16/3.281    # ft to m
    maze_height_meters = 14/3.281   # ft to m
    leftEdge_BlockZone_from_river_meters = 4/3.281
    width_BlockZone_meters = 0.52
    height_BlockZone_meters = 1.055
    # river_offset_meters = -.114
    river_offset_meters = 0
    river_width_meters = .85

    maze = np.zeros((meter_to_scale(maze_height_meters),meter_to_scale(maze_width_meters)))
    (maze_height, maze_width) = np.shape(maze)
    river_center = (int(maze_height/2),meter_to_scale(maze_width_meters/2+river_offset_meters))
    maze[:,0]=WALL_VALUE;maze[:,maze_width-1]=WALL_VALUE
    maze[0,:]=WALL_VALUE;maze[maze_height-1,:]=WALL_VALUE
    for k in range(maze_height):
        if abs(k/15.0-maze_height_meters/2)<river_width_meters/2:
            maze[k,river_center[1]]=WALL_VALUE
        else:
            maze[k,int(maze_width/2)]=WALL_VALUE
    leftEdge_BlockZone_from_leftEdgeMaze = meter_to_scale(leftEdge_BlockZone_from_river_meters+maze_width_meters/2+river_offset_meters)
    width_BlockZone = meter_to_scale(width_BlockZone_meters)
    height_BlockZone = meter_to_scale(height_BlockZone_meters)
    

    for i in range(leftEdge_BlockZone_from_leftEdgeMaze,leftEdge_BlockZone_from_leftEdgeMaze+width_BlockZone):
        for j in range(int(river_center[0]+height_BlockZone/(-2)),int(river_center[0]+height_BlockZone/(2))):
            maze[j,i] = WALL_VALUE

    # open by creating new file and append to it
    # with open('Labs/Final_Lab/Final_Lab_maze2.csv','w') as f:
    with open('Final_Lab/Final_Lab_maze2.csv','w') as f:
        np.savetxt(f, maze, delimiter=",", fmt="%d")