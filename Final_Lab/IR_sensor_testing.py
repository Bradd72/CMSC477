

import numpy as np
from robomaster import robot


if __name__ == '__main__':

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_sensor = ep_robot.