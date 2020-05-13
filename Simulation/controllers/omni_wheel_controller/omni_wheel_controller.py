"""omni_controller controller."""

from controller import Robot, LED
from enum import Enum, auto
from threading import Timer
import random


STATE_CHANGE_INTERVAL = 1
SPEED_FACTOR = 1.0

"""
CLASSES
"""


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

"""
INITS
"""

robot = Robot()
timestep = int(robot.getBasicTimeStep())

led = robot.getLED("led")
led.set(int("0xff0000", 16))


# Create an iterable list of the wheel motors
wheels = [
    robot.getMotor("wheel1"),
    robot.getMotor("wheel2"),
    robot.getMotor("wheel3")
]

for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)

# Create an iterable list of the distance sensors
ds = [
    robot.getDistanceSensor("ds0"),
    robot.getDistanceSensor("ds1"),
    robot.getDistanceSensor("ds2"),
    robot.getDistanceSensor("ds3"),
    robot.getDistanceSensor("ds4"),
    robot.getDistanceSensor("ds5"),
    robot.getDistanceSensor("ds6"),
    robot.getDistanceSensor("ds7"),
]

for sensor in ds:
    sensor.enable(timestep)


"""
FUNCTIONS
"""


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


def readSensors():
    # Map distance sensors into their respective values
    return map(lambda sensor: sensor.getValue(), ds)


"""
MAIN LOOP
"""

Timer(STATE_CHANGE_INTERVAL, increment_state).start()


while robot.step(timestep) != -1:
    move(move_direction)
    led.set(random.randint(16, (int("0xffffff", 16))))

    readSensors()
