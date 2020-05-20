"""omni_controller controller."""

import robot
import state_machine
import jobs

import random #only needed for LED demo, can be removed

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
    robot.led.set(random.randint(16, (int("0xffffff", 16)))) #only test/demo, can be removed

    # robot.transmitter.send("Hello from omnibot".encode())
    # if robot.receiver.getQueueLength() != 0:
        # print("rx:", robot.receiver.getData()," strength:", robot.receiver.getSignalStrength(), " direction:", robot.receiver.getEmitterDirection())

    
