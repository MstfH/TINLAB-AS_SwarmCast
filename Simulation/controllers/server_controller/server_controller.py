"""sever_controller controller."""

from controller import Robot
import pickle
from math import sqrt
import numpy as np
from scipy.optimize import linear_sum_assignment
import sys

from Bot import Bot
sys.path.append('..')
from stateDefs import ServerState as ServerState
from stateDefs import BotState as BotState

TIME_STEP = 32
POS_TOLERANCE = 0.05
GRID_SIZE = 3
GRID_ORIGIN = 0
GRID_SPACING = 0.5
MAX_SHELL = (GRID_SIZE - 1) / 2
GRID_POSITIONS = []


for x in range(GRID_SIZE):
    for y in range(GRID_SIZE):
        GRID_POSITIONS.append([
            GRID_ORIGIN + x * GRID_SPACING,
            GRID_ORIGIN + y * GRID_SPACING
        ])

GRID_POSITIONS = np.array(GRID_POSITIONS)
print(GRID_POSITIONS)

state = ServerState.WAITING_FOR_CONNECTIONS
current_shell = 0

bots = []

robot = Robot()
emitter = robot.getEmitter("emitter")
receiver = robot.getReceiver("receiver")
receiver.enable(100)


def get_shell(point):
    x, y = point
    center = GRID_ORIGIN + (GRID_SIZE - 1) / 2 * GRID_SPACING
    delta_x = abs(center - x)
    delta_y = abs(center - y)
    max_delta = max(delta_x, delta_y)
    steps = max_delta / GRID_SPACING
    return steps


def calculate_optimal_assignment():
    global state
    cost = np.empty([len(bots), len(bots)])
    for i in range(len(bots)):
        for j in range(len(bots)):
            cost[i, j] = np.sum(np.absolute(bots[i].position - GRID_POSITIONS[j]))

    rows, cols = linear_sum_assignment(cost)
    for (row_i, col_i) in zip(rows, cols):
        bot = bots[row_i]
        target = GRID_POSITIONS[col_i]
        bot.set_target(target)
        bot.set_shell(get_shell(target))
    
    state = ServerState.WAITING_FOR_FORMATION


def send_message(message):
    emitter.send(pickle.dumps(message))


def get_state(bot):
    pos_x, pos_y = bot.position
    target_x, target_y = bot.target

    if bot.shell != current_shell:
        return BotState.IDLE

    if pos_y - target_y > POS_TOLERANCE:
        return BotState.TRAVELLING_NORTH

    if target_x - pos_x > POS_TOLERANCE:
        return BotState.TRAVELLING_EAST

    if target_y - pos_y > POS_TOLERANCE:
        return BotState.TRAVELLING_SOUTH

    if pos_x - target_x > POS_TOLERANCE:
        return BotState.TRAVELLING_WEST

    return BotState.IN_FORMATION


def current_shell_in_formation():
    bots_in_current_shell = [bot for bot in bots if bot.shell == current_shell]
    return all(bot.state == BotState.IN_FORMATION for bot in bots_in_current_shell)


if GRID_SIZE % 2 == 0:
    print("WARNING: Please set GRID_SIZE to an uneven number.")

while robot.step(TIME_STEP) != -1:
    if len(bots) > GRID_SIZE**2:
        print("WARNING: Excess of bots.")

    while receiver.getQueueLength() > 0:
        bot = None
        raw_data = receiver.getData()
        (id, position) = pickle.loads(raw_data)
        emitter.setChannel(id)

        if id in [bot.id for bot in bots]:
            bot = next(bot for bot in bots if bot.id == id)
            bot.set_position(np.array(position))
            bot.set_state(get_state(bot))

        else:
            bot = Bot(id=id, position=np.array(position))
            bots.append(bot)

        send_message(bot.state)

        if state == ServerState.WAITING_FOR_CONNECTIONS and len(bots) == GRID_SIZE**2:
            state = ServerState.CALCULATING_OPTIMAL_ASSIGNMENT
            calculate_optimal_assignment()

        if current_shell_in_formation():
            if current_shell == MAX_SHELL:
                state = ServerState.ASSIGNING_COLORS

            if state == ServerState.WAITING_FOR_FORMATION:
                current_shell += 1

        receiver.nextPacket()
