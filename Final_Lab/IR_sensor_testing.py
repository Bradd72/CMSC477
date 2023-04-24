

import numpy as np
from robomaster import robot
import time


def sub_distance_handler(dist_info):
    print(dist_info)


if __name__ == '__main__':

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_sensor = ep_robot.sensor


    ep_sensor.sub_distance(freq=10,callback=sub_distance_handler)


    time.sleep(10)

    ep_sensor.unsub_distance()

    ep_robot.close()