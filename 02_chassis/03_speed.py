# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")

    ep_chassis = ep_robot.chassis

    x_val = 0.5
    y_val = 0.3
    z_val = 30

    # Move forward 3 seconds
    ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
    time.sleep(3)

    # Move backward 3 seconds
    ep_chassis.drive_speed(x=-x_val, y=0, z=0, timeout=5)
    time.sleep(3)

    # Move left 3 seconds
    ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=5)
    time.sleep(3)

    # Move right 3 seconds
    ep_chassis.drive_speed(x=0, y=y_val, z=0, timeout=5)
    time.sleep(3)

    # Turn left 3 seconds
    ep_chassis.drive_speed(x=0, y=0, z=-z_val, timeout=5)
    time.sleep(3)

    # Turn right 3 seconds
    ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
    time.sleep(3)

    # Stop
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

    ep_robot.close()
