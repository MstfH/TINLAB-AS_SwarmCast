"""omni_controller controller."""

from controller import Robot, Emitter, Receiver
from enum import Enum, auto
from threading import Timer

STATE_CHANGE_INTERVAL = 1
SPEED_FACTOR = 4.0


class Direction(Enum):
    STATIONARY = auto()
    NORTH = auto()
    EAST = auto()
    SOUTH = auto()
    WEST = auto()


DIRECTION_ORDER = [
    Direction.STATIONARY,
    Direction.NORTH,
    Direction.EAST,
    Direction.SOUTH,
    Direction.WEST
]


direction_index = 0
move_direction = DIRECTION_ORDER[direction_index]


robot = Robot()

transmitter = robot.getEmitter("transmitter")
receiver = robot.getReceiver("receiver")
receiver.enable(100) #sampling period

wheels = [
    robot.getMotor("wheel1"),
    robot.getMotor("wheel2"),
    robot.getMotor("wheel3")
]

for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)

timestep = int(robot.getBasicTimeStep())


def freezeWheels():
    for wheel in wheels:
        wheel.setVelocity(0)


def moveNorth():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(-SPEED_FACTOR)
    wheels[2].setVelocity(SPEED_FACTOR)


def moveEast():
    wheels[0].setVelocity(-2 * SPEED_FACTOR)
    wheels[1].setVelocity(SPEED_FACTOR)
    wheels[2].setVelocity(SPEED_FACTOR)


def moveSouth():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(SPEED_FACTOR)
    wheels[2].setVelocity(-SPEED_FACTOR)


def moveWest():
    wheels[0].setVelocity(2 * SPEED_FACTOR)
    wheels[1].setVelocity(-SPEED_FACTOR)
    wheels[2].setVelocity(-SPEED_FACTOR)


direction_map = {
    Direction.STATIONARY: freezeWheels,
    Direction.NORTH: moveNorth,
    Direction.EAST: moveEast,
    Direction.SOUTH: moveSouth,
    Direction.WEST: moveWest
}


def move(direction):
    move_func = direction_map[direction]
    move_func()


def increment_state():
    global move_direction, direction_index
    direction_index = (direction_index + 1) % len(DIRECTION_ORDER)
    move_direction = DIRECTION_ORDER[direction_index]
    Timer(STATE_CHANGE_INTERVAL, increment_state).start()

###Timer freezes this code. Disabling breaks movement
""""
Timer(STATE_CHANGE_INTERVAL, increment_state).start()
"""
while robot.step(timestep) != -1:
    move(move_direction)
    transmitter.send("Hello from omnibot".encode())
    if receiver.getQueueLength() != 0:
        print("rx:", receiver.getData()," strength:", receiver.getSignalStrength(), " direction:", receiver.getEmitterDirection())

    