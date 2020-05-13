"""omni_controller controller."""

from controller import Robot, LED
import math
from enum import Enum, auto
from threading import Timer
import random

# Simple default noop function, nothing special


def noop():
    return None


SPEED_FACTOR = 5.0


# The state enum essentially dictates what states the bot can be in
# Separate functions are responsible for execting logic based on what state is current active


class State(Enum):
    IDLE = auto()
    DECELERATING = auto()
    TRAVELING_NORTH = auto()
    TRAVELING_EAST = auto()
    TRAVELING_SOUTH = auto()
    TRAVELING_WEST = auto()

# Every job has its own unique enum to distinguish between them


class Job(Enum):
    DRIVE_SQUARE = auto()


# Here we are essentially just initializing the robot, its sensors and actuators
robot = Robot()
timestep = int(robot.getBasicTimeStep())

led = robot.getLED("led")

accelerometer = robot.getAccelerometer("accelerometer")
accelerometer.enable(timestep)


# Create an iterable list of the motors/wheels
wheels = [
    robot.getMotor("wheel1"),
    robot.getMotor("wheel2"),
    robot.getMotor("wheel3")
]

for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)

# Create an iterable list of the distance sensors
ds = []
dsNames = [
    'ds0', 'ds1', 'ds2', 'ds3',
    'ds4', 'ds5', 'ds6', 'ds7'
]

for i in range(8):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(timestep)


def decelerate():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(0)


def travel_north():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(-SPEED_FACTOR)
    wheels[2].setVelocity(SPEED_FACTOR)


def travel_east():
    wheels[0].setVelocity(-2 * SPEED_FACTOR)
    wheels[1].setVelocity(SPEED_FACTOR)
    wheels[2].setVelocity(SPEED_FACTOR)


def travel_south():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(SPEED_FACTOR)
    wheels[2].setVelocity(-SPEED_FACTOR)


def travel_west():
    wheels[0].setVelocity(2 * SPEED_FACTOR)
    wheels[1].setVelocity(-SPEED_FACTOR)
    wheels[2].setVelocity(-SPEED_FACTOR)


def readSensors():
    # reads sensors and returns list of sensor values
    dsValues = []
    for i in range(8):
        dsValues.append(ds[i].getValue())
    return dsValues


# Calculate the total acceleration in any non-vertical direction based on the Pythagoras theorem
def get_total_acceleration():
    (x, y, z) = accelerometer.getValues()
    return math.sqrt(abs(x)**2 + abs(z)**2)


state = State.IDLE

# Every time this generator function is called, the next direction state is yielded.
# North will be returned after West has been returned


def get_next_direction():
    while(True):
        yield State.TRAVELING_NORTH
        yield State.TRAVELING_EAST
        yield State.TRAVELING_SOUTH
        yield State.TRAVELING_WEST


# We need to instantiate the generator function, so that the function state can be saved to memory
next_direction_generator = get_next_direction()


# A generator function that will yield the FSM state required to drive in a rectangular shape
# Also instantiated so that function state can be saved to memory
def drive_square():
    time_since = 0
    while(True):
        if(time_since < 2000):
            time_since += timestep
            yield state
        else:
            while(True):
                time_since = 0
                yield State.DECELERATING
                acceleration = get_total_acceleration()
                print(acceleration)
                if(-0.05 < acceleration < 0.05):
                    yield next(next_direction_generator)
                    break

# We get the newly calculated FSM state based on what the state_generator yields
# state_generator can be seen as active_job_instance


def get_state():
    new_state = next(state_generator)
    return new_state

# Execute some function based on the current state of the FSM


def execute_state():
    state_map = {
        State.IDLE: noop,
        State.DECELERATING: decelerate,
        State.TRAVELING_NORTH: travel_north,
        State.TRAVELING_EAST: travel_east,
        State.TRAVELING_SOUTH: travel_south,
        State.TRAVELING_WEST: travel_west
    }
    state_func = state_map[state]
    state_func()


state_generator = drive_square()

while robot.step(timestep) != -1:
    readSensors()

    state = get_state()
    execute_state()
    led.set(random.randint(16, (int("0xffffff", 16))))
