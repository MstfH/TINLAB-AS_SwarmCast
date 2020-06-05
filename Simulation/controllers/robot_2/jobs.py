from enum import Enum, auto
import math

import robot
import state_machine
import time


# Every job has its own unique enum to distinguish between them


class Job(Enum):
    WAIT = auto()
    DRIVE_SQUARE = auto()
    AVOID = auto()

# Calculate the total acceleration in any non-vertical direction based on the Pythagoras theorem


def get_total_acceleration():
    (x, y, z) = robot.accelerometer.getValues()
    return math.sqrt(x**2 + z**2)


def is_stationary():
    return get_total_acceleration() == 0


def wait():
    while(True):
        if(is_stationary()):
            return state_machine.State.IDLE
        else:
            return state_machine.State.DECELERATING


# Every time this generator function is called, the next direction state is yielded.
# North will be returned after West has been returned
def get_next_direction():
    while(True):
        yield state_machine.State.TRAVELING_NORTH
        yield state_machine.State.TRAVELING_EAST
        yield state_machine.State.TRAVELING_SOUTH
        yield state_machine.State.TRAVELING_WEST


# We need to instantiate the generator function, so that the function state can be saved to memory
next_direction_generator = get_next_direction()


# A generator function that will yield the FSM state required to drive in a rectangular shape
# Also instantiated so that function state can be saved to memory
def drive_square():
    time_since = 0
    while(True):
        if(time_since < 2000):
            time_since += robot.timestep
            yield state_machine.state
        else:
            while(True):
                time_since = 0
                yield state_machine.State.DECELERATING
                acceleration = get_total_acceleration()
                if(-0.05 < acceleration < 0.05):
                    yield next(next_direction_generator)
                    break


movementDirection = 'd'


def avoid():
    while True:
        if movementDirection == 'west':
            # code crashes at this line
            yield state_machine.State.TRAVELING_WEST
            if -0.05 < get_total_acceleration() < 0.05:
                yield next(next_direction_generator)
                break
            time.sleep(0.2)
            # After crash it finishes everything that comes after this
            break
        if movementDirection == 'east':
            yield state_machine.State.TRAVELING_EAST
            if -0.05 < get_total_acceleration() < 0.05:
                yield next(next_direction_generator)
                break
            time.sleep(0.2)
            break
        if movementDirection == 'north':
            yield state_machine.State.TRAVELING_NORTH
            if -0.05 < get_total_acceleration() < 0.05:
                yield next(next_direction_generator)
                break
            time.sleep(0.2)
            break
        if movementDirection == 'south':
            yield state_machine.State.TRAVELING_SOUTH
            if -0.05 < get_total_acceleration() < 0.05:
                yield next(next_direction_generator)
                break
            time.sleep(0.2)
            break


job_map = {
    Job.WAIT: wait,
    Job.DRIVE_SQUARE: drive_square,
    Job.AVOID: avoid
}


def set_job(job_type):
    global job
    new_job = job_map[job_type]
    job = new_job()


job = drive_square()
