"""sever_controller controller."""

from controller import Supervisor
import pickle
import random
from math import sqrt
import numpy as np
from scipy.optimize import linear_sum_assignment
import sys
from PIL import Image
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

for x in range(GRID_SIZE):
    for y in range(GRID_SIZE):
        GRID_POSITIONS.append([
            GRID_ORIGIN + x * GRID_SPACING,
            GRID_ORIGIN + y * GRID_SPACING
        ])

GRID_POSITIONS = np.array(GRID_POSITIONS)
print(GRID_POSITIONS)

PIXEL_MAP = {}
IMAGE_NAME = "flag.png"
image = Image.open(IMAGE_NAME)
IMAGE_DATA = np.asarray(image)
IMAGE_DATA = IMAGE_DATA.reshape(GRID_SIZE**2, 3)

for color_value, position in zip(IMAGE_DATA, GRID_POSITIONS):
    color = '0x%02x%02x%02x' % tuple(color_value)
    PIXEL_MAP.update({tuple(position): color})
    
server_state = ServerState.WAITING_FOR_CONNECTIONS

current_shell = 0

bots = []

supervisor = Supervisor()
root = supervisor.getRoot()
root_children = root.getField("children")

for i in range(GRID_SIZE**2):
    x = random.randint(-20, 20) / 10
    z = random.randint(-20, 20) / 10
    root_children.importMFNodeFromString(-1, f"OmniBot {{translation {x} 0.06 {z}}}")

emitter = supervisor.getEmitter("emitter")
receiver = supervisor.getReceiver("receiver")
receiver.enable(100)

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
        bot.set_color(PIXEL_MAP.get(tuple(target)))
    
    server_state = ServerState.WAITING_FOR_FORMATION


def send_message(message):
    emitter.send(pickle.dumps(message))

# Determine whether 2 bots have swapped in the past
def have_swapped(b1, b2):
    return b2 in b1.swapped_with or b1 in b2.swapped_with

# Swap the target, state and shell properties of 2 bots
def swap(b1, b2):
    target = b1.target
    shell = b1.shell
    color = b1.color

    b1.set_target(b2.target)
    b1.set_shell(b2.shell)
    b1.set_color(b2.color)
    b1.append_swapped(b2)
    
    b2.set_target(target)
    b2.set_shell(shell)
    b2.set_color(color)
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

while supervisor.step(TIME_STEP) != -1:

    while receiver.getQueueLength() > 0:
        bot = None
        raw_data = receiver.getData()
        (id, position) = pickle.loads(raw_data)
        emitter.setChannel(id)
            
        if id in [bot.id for bot in bots]:
            if server_state != ServerState.WAITING_FOR_CONNECTIONS:
                bot = next(bot for bot in bots if bot.id == id)
                bot.set_position(np.array(position))
                bot.set_state(get_state(bot))
                downlink_data = (bot.state, bot.color)
                send_message(downlink_data)

        else:
            bot = Bot(id=id, position=np.array(position))
            bots.append(bot)

        receiver.nextPacket()

    if server_state == ServerState.WAITING_FOR_CONNECTIONS and len(bots) == GRID_SIZE**2:
        server_state = ServerState.CALCULATING_OPTIMAL_ASSIGNMENT
        calculate_optimal_assignment()

    if current_shell_in_formation():
        if current_shell == MAX_SHELL:
            server_state = ServerState.DONE

        if server_state == ServerState.WAITING_FOR_FORMATION:
            current_shell += 1

