"""omni_controller controller."""

import random

import robot
import state_machine
import jobs
import lidarLogic as collision


def readSensors():
    # reads sensors and returns list of sensor values
    dsValues = []
    for i in range(8):
        dsValues.append(robot.ds[i].getValue())
    #lidar.getRangeImage() for all layers
    #lidar.getLayerRangeImage(3) for layer 3 of the n layers
    #where layer 1 is the lowest and top layer the highest
    collision.lidarPoint(robot.lidar.getRangeImage())
    return dsValues


while robot.step(robot.timestep) != -1:
    readSensors()

    state_machine.state = state_machine.get_next_state()
    state_machine.execute_state()
    robot.led.set(random.randint(16, (int("0xffffff", 16))))