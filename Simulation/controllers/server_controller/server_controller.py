"""sever_controller controller."""

from controller import Robot
import pickle
from math import sqrt
import numpy as np
from scipy.optimize import linear_sum_assignment
import sys

from Bot import Bot
sys.path.append('..')
from stateDefs import BotState as BotState
from stateDefs import ServerState as ServerState

TIME_STEP = 32
POS_TOLERANCE = 0.05
GRID_SIZE = 3
GRID_ORIGIN = 0.5
GRID_SPACING = 0.5
MAX_SHELL = (GRID_SIZE - 1) / 2
GRID_POSITIONS = []

HEADING_CORR_TOLERANCE = 0.10

for x in range(GRID_SIZE):
    for y in range(GRID_SIZE):
        GRID_POSITIONS.append([
            GRID_ORIGIN + x * GRID_SPACING,
            GRID_ORIGIN + y * GRID_SPACING
        ])

GRID_POSITIONS = np.array(GRID_POSITIONS)
print(GRID_POSITIONS)

server_state = ServerState.WAITING_FOR_CONNECTIONS
current_shell = 0

bots = []

robot = Robot()
emitter = robot.getEmitter("emitter")
receiver = robot.getReceiver("receiver")
receiver.enable(TIME_STEP)

# Simple pythagorean distance between 2 points
def distance_between(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

# Determine how many steps a given point on the grid is removed from its center
def get_shell(point):
    x, y = point
    center = GRID_ORIGIN + (GRID_SIZE - 1) / 2 * GRID_SPACING
    delta_x = abs(center - x)
    delta_y = abs(center - y)
    max_delta = max(delta_x, delta_y)
    steps = max_delta / GRID_SPACING
    return steps

# Determine what the optimal assignment for assigning bots to positions on the grid is,
# So that the total cummulative distance traveled is minimal
# For this, we use the Hungarian method provided by scipy through linear_sum_assignment
def calculate_optimal_assignment():
    global server_state
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
    
    server_state = ServerState.WAITING_FOR_FORMATION


def send_message(message):
    emitter.send(pickle.dumps(message))

# Determine whether 2 bots have swapped in the past
def have_swapped(b1, b2):
    return b2 in b1.swapped_with or b1 in b2.swapped_with

# Swap the target, state and shell properties of 2 bots
def swap(b1, b2):
    target = b1.target
    state = b1.state
    shell = b1.shell

    b1.set_target(b2.target)
    b1.set_state(b2.state)
    b1.set_shell(b2.shell)
    b1.append_swapped(b2)
    
    b2.set_target(target)
    b2.set_state(state)
    b2.set_shell(shell)
    b2.append_swapped(b1)

# Determine the next state for a bot, based on its own and other bot's properties
def get_state(bot):
    pos_x, pos_y = bot.position
    target_x, target_y = bot.target

    close_bots = [other_bot for other_bot in bots
                  if bot.id != other_bot.id
                  and distance_between(bot.position, other_bot.position) < 0.4]

    for other_bot in close_bots:
        if not have_swapped(bot, other_bot):
            swap(bot, other_bot)

    if heading < (0 - HEADING_CORR_TOLERANCE):
        return BotState.TURNING_CCW
    
    if heading > (0 + HEADING_CORR_TOLERANCE):
        return BotState.TURNING_CW

    if bot.shell > current_shell:
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

# Determine whether all bots that belong to the current shell are currently in formation
def current_shell_in_formation():
    bots_in_current_shell = [bot for bot in bots if bot.shell == current_shell]
    return all(bot.state == BotState.IN_FORMATION for bot in bots_in_current_shell)


if GRID_SIZE % 2 == 0:
    raise Exception("GRID_SIZE should be set to be an even number.")

while robot.step(TIME_STEP) != -1:
    if len(bots) > GRID_SIZE**2:
        raise Exception("Too many bots.")

    while receiver.getQueueLength() > 0:
        bot = None
        raw_data = receiver.getData()
        (id, message) = pickle.loads(raw_data)
        position = message.get("position")
        heading = message.get("heading")
        emitter.setChannel(id)
            
        if id in [bot.id for bot in bots]:
            if server_state != ServerState.WAITING_FOR_CONNECTIONS:
                bot = next(bot for bot in bots if bot.id == id)
                bot.set_position(np.array(position))
                bot.set_heading(heading)
                bot.set_state(get_state(bot))
                send_message(bot.state)

        else:
            bot = Bot(id=id, position=np.array(position))
            bots.append(bot)

        receiver.nextPacket()

    if server_state == ServerState.WAITING_FOR_CONNECTIONS and len(bots) == GRID_SIZE**2:
        server_state = ServerState.CALCULATING_OPTIMAL_ASSIGNMENT
        calculate_optimal_assignment()

    if current_shell_in_formation():
        if current_shell == MAX_SHELL:
            server_state = ServerState.ASSIGNING_COLORS

        if server_state == ServerState.WAITING_FOR_FORMATION:
            current_shell += 1

