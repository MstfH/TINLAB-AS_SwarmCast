"""robot_2 controller."""

import random

import robot
import aggregation


def readSensors():
    # reads sensors and returns list of sensor values
    dsValues = []
    for i in range(8):
        dsValues.append(robot.ds[i].getValue())
    return dsValues


while robot.step(robot.timestep) != -1:

    aggregation.run()
    print("Aggregation goal reached!")

    robot.led.set(random.randint(16, (int("0xffffff", 16))))
