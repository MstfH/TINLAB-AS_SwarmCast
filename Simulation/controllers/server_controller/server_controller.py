"""sever_controller controller."""

from controller import Robot
import pickle
from math import sqrt
import numpy as np
from scipy.optimize import linear_sum_assignment
from enum import Enum, auto


class State(Enum):
    WAITING_FOR_CONNECTIONS = auto()
    CALCULATING_OPTIMAL_ASSIGNMENT = auto()
    WAITING_FOR_FORMATION = auto()


TRAVELING_NORTH = "N"
TRAVELING_EAST = "E"
TRAVELING_SOUTH = "S"
TRAVELING_WEST = "W"
IDLE = "I"

TIME_STEP = 32
POS_TOLERANCE = 0.05
GRID_WIDTH = 3
GRID_HEIGHT = 3
GRID_ORIGIN_X = 0
GRID_ORIGIN_Y = 0
GRID_SPACING = 0.5
GRID_POSITIONS = []


for x in range(GRID_WIDTH):
    for y in range(GRID_HEIGHT):
        GRID_POSITIONS.append([
            GRID_ORIGIN_X + x * GRID_SPACING,
            GRID_ORIGIN_Y + y * GRID_SPACING
        ])

GRID_POSITIONS = np.array(GRID_POSITIONS)
print(GRID_POSITIONS)

state = State.WAITING_FOR_CONNECTIONS

bots = []
bot_ids = []

robot = Robot()
emitter = robot.getEmitter("emitter")
receiver = robot.getReceiver("receiver")
receiver.enable(100)


def calculate_optimal_assignment():
    cost = np.empty([len(bots), len(bots)])
    for i in range(len(bots)):
        for j in range(len(bots)):
            cost[i, j] = np.sum(np.absolute(
                bots[i].get("position") - GRID_POSITIONS[j]))

    rows, cols = linear_sum_assignment(cost)
    for (row_i, col_i) in zip(rows, cols):
        bots[row_i].update({"target": GRID_POSITIONS[col_i]})


def send_message(message):
    emitter.send(pickle.dumps(message))


def get_state(position, target):
    pos_x, pos_y = position
    target_x, target_y = target
    if pos_y - target_y > POS_TOLERANCE:
        return TRAVELING_NORTH

    if target_x - pos_x > POS_TOLERANCE:
        return TRAVELING_EAST

    if target_y - pos_y > POS_TOLERANCE:
        return TRAVELING_SOUTH

    if pos_x - target_x > POS_TOLERANCE:
        return TRAVELING_WEST

    return IDLE


while robot.step(TIME_STEP) != -1:
    if len(bots) > GRID_WIDTH * GRID_HEIGHT:
        print("WARNING: Excess of bots")

    while receiver.getQueueLength() > 0:
        raw_data = receiver.getData()
        (id, position) = pickle.loads(raw_data)
        emitter.setChannel(id)

        if id in bot_ids:
            bot = next(bot for bot in bots if bot.get("id") == id)
            bot.update({"position": position})
            new_state = get_state(
                bot.get("position"), bot.get("target")
            )
            send_message(new_state)

        else:
            print(f"Registered bot {id} @ {position}")
            bots.append(
                {"id": id, "position": np.array(position), "target": None}
            )
            bot_ids.append(id)
            send_message(IDLE)

        if state == State.WAITING_FOR_CONNECTIONS and len(bots) == GRID_WIDTH * GRID_HEIGHT:
            state = State.CALCULATING_OPTIMAL_ASSIGNMENT
            calculate_optimal_assignment()

        receiver.nextPacket()
