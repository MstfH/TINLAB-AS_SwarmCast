"""omni_controller controller."""

import random

import robot
import state_machine
import jobs
import threading
import distanceSensorLogic as dSLogic


def readSensors():
    # reads sensors and returns list of sensor values
    dsValues = []
    for i in range(8):
        dsValues.append(robot.ds[i].getValue())
    thread = threading.Thread(target=dSLogic.dSPoints(dsValues, state_machine.state))
    thread.start()
    return dsValues

def main():
    while robot.step(robot.timestep) != -1:
        readSensors()
        
        state_machine.state = state_machine.get_next_state()
        state_machine.execute_state()
        robot.led.set(random.randint(16, (int("0xffffff", 16))))

while True:
    #this way it crashes every loop
    try:
        main()
    except StopIteration:
        print('good')
        continue 