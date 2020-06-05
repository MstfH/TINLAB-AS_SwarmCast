from enum import Enum, auto

import robot
import jobs

SPEED_FACTOR = 5.0


# Simple default noop function, nothing special


def noop():
    return None

# The state enum essentially dictates what states the bot can be in
# Separate functions are responsible for execting logic based on what state is current active


class State(Enum):
    IDLE = auto()
    DECELERATING = auto()
    TRAVELING_NORTH = auto()
    TRAVELING_EAST = auto()
    TRAVELING_SOUTH = auto()
    TRAVELING_WEST = auto()


def decelerate():
    robot.wheels[0].setVelocity(0)
    robot.wheels[1].setVelocity(0)
    robot.wheels[2].setVelocity(0)


def travel_north():
    robot.wheels[0].setVelocity(0)
    robot.wheels[1].setVelocity(-SPEED_FACTOR)
    robot.wheels[2].setVelocity(SPEED_FACTOR)


def travel_east():
    robot.wheels[0].setVelocity(-2 * SPEED_FACTOR)
    robot.wheels[1].setVelocity(SPEED_FACTOR)
    robot.wheels[2].setVelocity(SPEED_FACTOR)


def travel_south():
    robot.wheels[0].setVelocity(0)
    robot.wheels[1].setVelocity(SPEED_FACTOR)
    robot.wheels[2].setVelocity(-SPEED_FACTOR)


def travel_west():
    robot.wheels[0].setVelocity(2 * SPEED_FACTOR)
    robot.wheels[1].setVelocity(-SPEED_FACTOR)
    robot.wheels[2].setVelocity(-SPEED_FACTOR)


state_map = {
    State.IDLE: noop,
    State.DECELERATING: decelerate,
    State.TRAVELING_NORTH: travel_north,
    State.TRAVELING_EAST: travel_east,
    State.TRAVELING_SOUTH: travel_south,
    State.TRAVELING_WEST: travel_west
}

# We get the newly calculated FSM state based on what the current job yields


def get_next_state():
    new_state = next(jobs.job)
    return new_state

# Execute some function based on the current state of the FSM


def execute_state():
    state_func = state_map[state]
    state_func()


state = State.IDLE
