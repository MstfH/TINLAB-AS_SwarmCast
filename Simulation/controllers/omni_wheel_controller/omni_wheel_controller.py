"""omni_controller controller."""

import random

import robot
import state_machine
import jobs


def readSensors():
    # reads sensors and returns list of sensor values
    dsValues = []
    for i in range(8):
        dsValues.append(robot.ds[i].getValue())
    return dsValues


while robot.step(robot.timestep) != -1:
    readSensors()

    state_machine.state = state_machine.get_next_state()
    state_machine.execute_state()
    robot.led.set(random.randint(16, (int("0xffffff", 16))))
