"""sever_controller controller."""

from controller import Robot
import collisionDetection as cd
from controller import Supervisor
import pickle
import random
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
import sys
from PIL import Image
from Bot import Bot

sys.path.append('..')
from stateDefs import BotState as BotState
from stateDefs import ServerState as ServerState

PIXEL_MAP = {}
IMAGE_NAME = "rainbow.png"
IMAGE = Image.open(IMAGE_NAME)
IMAGE_WIDTH, IMAGE_HEIGHT = IMAGE.size

if IMAGE_WIDTH != IMAGE_HEIGHT:
    raise Exception("Error: Image is not rectangular")

if IMAGE_WIDTH % 2 != 1 or IMAGE_HEIGHT %  2 != 1:
    raise Exception("Error: Image width and height must both be an uneven number")

IMAGE_DATA = np.asarray(IMAGE)
IMAGE_DATA = IMAGE_DATA.reshape(IMAGE_WIDTH**2, 3)
TIME_STEP = 32
GRID_SIZE = IMAGE_WIDTH
MAX_SHELL = (GRID_SIZE - 1) / 2
GRID_POSITIONS = []


POS_TOLERANCE = 0.05             #deviation tolerance for target
GRID_SPACING = 0.8               #distance between bots
GRID_CENTER = None               #you may optionally set this to the desired center of the grid
SWAP_LIMIT = 0.4                 #distance at which bots may swap
ENABLE_SHELLING = False          #whether to use shelling. shelling improves grid assimilation times when GRID_SPACING is very small
HEADING_CORR_TOLERANCE = 0.10    #tolerance between a bot's heading and absolute north
BOT_SPREAD = 0.5                 #number from 0 to 1 to determine dispersion of bots   
#LED_SIZE = GRID_SPACING          #size of the LED on the bot
LED_SIZE = 0.1          #size of the LED on the bot

server_state = ServerState.WAITING_FOR_CONNECTIONS

current_shell = 0

bots = []

supervisor = Supervisor()
root = supervisor.getRoot()
root_children = root.getField("children")

emitter = supervisor.getEmitter("emitter")
receiver = supervisor.getReceiver("receiver")
receiver.enable(TIME_STEP)

# Simple pythagorean distance between 2 points
def distance_between(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_random_coordinates():
    coordinate_buffer = []
    while(True):
        x = random.randint(int(-45 * BOT_SPREAD), int(45 * BOT_SPREAD)) / 10 
        y = random.randint(int(-45 * BOT_SPREAD), int(45 * BOT_SPREAD)) / 10
        if len([c for c in coordinate_buffer if distance_between(c, (x, y)) < 0.5]) == 0:
            coordinate_buffer.append((x, y))
            yield (x, y)

#spawn required bots at random positions
coordinate_generator = get_random_coordinates()
for i in range(GRID_SIZE**2):
    x, y = next(coordinate_generator)
    root_children.importMFNodeFromString(-1, f"OmniBot {{translation {x} 0.06 {y} led_size {LED_SIZE} 0.01 {LED_SIZE}}}")

# Determine how many steps a given point on the grid is removed from its center
def get_shell(point):
    x, y = point
    center_x, center_y = GRID_CENTER
    delta_x = abs(center_x - x)
    delta_y = abs(center_y - y)
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

# Serialize data and transmit message to bot
def send_message(message):
    emitter.send(pickle.dumps(message))

# Determine whether 2 bots have swapped in the past
def have_swapped(b1, b2):
    return b2 in b1.swapdeque or b1 in b2.swapdeque

# Swap the target, state and shell properties of 2 bots
def swap(b1, b2):

    if b1 not in b2.swapdeque:
        target = b1.target
        shell = b1.shell
        color = b1.color

        b1.set_target(b2.target)
        b1.set_shell(b2.shell)
        b1.set_color(b2.color)
        b1.append_swapdeque(b2)
        
        b2.set_target(target)
        b2.set_shell(shell)
        b2.set_color(color)
        b2.append_swapdeque(b1)

# Determine the next state for a bot, based on its own and other bot's properties
def get_state(bot):
    pos_x, pos_y = bot.position
    target_x, target_y = bot.target

    close_bots = [other_bot for other_bot in bots
                  if bot.id != other_bot.id
                  and distance_between(bot.position, other_bot.position) < SWAP_LIMIT]

    for other_bot in close_bots:
        if not have_swapped(bot, other_bot) and (bot.shell >= current_shell and other_bot.shell >= current_shell) if ENABLE_SHELLING else True:
            swap(bot, other_bot)

    if heading < (0 - HEADING_CORR_TOLERANCE):
        return BotState.TURNING_CCW
    
    if heading > (0 + HEADING_CORR_TOLERANCE):
        return BotState.TURNING_CW

    if bot.shell > current_shell and ENABLE_SHELLING:
        return BotState.IDLE

    movement_candidates = []
    if target_y < pos_y and pos_y - target_y > POS_TOLERANCE:
        movement_candidates.append(BotState.TRAVELLING_NORTH)

    if target_x > pos_x and target_x - pos_x > POS_TOLERANCE:
        movement_candidates.append(BotState.TRAVELLING_EAST)

    if target_y > pos_y and target_y - pos_y > POS_TOLERANCE:
        movement_candidates.append(BotState.TRAVELLING_SOUTH)

    if target_x < pos_x and pos_x - target_x > POS_TOLERANCE:
        movement_candidates.append(BotState.TRAVELLING_WEST)

    if len(movement_candidates) > 0:
        return random.choice(movement_candidates)

    return BotState.IN_FORMATION

# Determine whether all bots that belong to the current shell are currently in formation
def current_shell_in_formation():
    bots_in_current_shell = [bot for bot in bots if bot.shell == current_shell]
    return all(bot.state == BotState.IN_FORMATION for bot in bots_in_current_shell)

# Determine whether all bots are in formation
def all_bots_in_formation():
    return all(bot.state == BotState.IN_FORMATION for bot in bots)

# Shift the target of all bots by a certain delta
def offset_all_bots(point):
    point_x, point_y = point
    for bot in bots:
        target_x, target_y = bot.target
        bot.target = (target_x + point_x, target_y + point_y)

# Generator function to iterate through directions sequentially
def get_next_offset():
    i = 0
    while True:
        offset = [math.cos(i * math.pi)  * 0.5, math.sin(i * math.pi) * 0.5]
        i = i + 0.5
        yield offset

offset_generator = get_next_offset()

# Calculate average of a list of numbers
def avg(nums):
    return sum(nums) / len(nums)

# Initialize the grid and assign colors once all bots have connected
def construct_grid():
    global GRID_CENTER, GRID_POSITIONS
    
    if GRID_CENTER == None:
        center_x = avg([bot.position[0] for bot in bots])
        center_y = avg([bot.position[1] for bot in bots])
        GRID_CENTER = (center_x, center_y)

    n_outer_layers = int((GRID_SIZE - 1) / 2)
    for x in range(-n_outer_layers, n_outer_layers  + 1):
        for y in range(-n_outer_layers, n_outer_layers + 1):
            GRID_POSITIONS.append([
                center_x + x * GRID_SPACING,
                center_y + y * GRID_SPACING
            ])
    GRID_POSITIONS = np.array(GRID_POSITIONS)
    print(GRID_POSITIONS)

    for color_value, position in zip(IMAGE_DATA, GRID_POSITIONS):
        color = '0x%02x%02x%02x' % tuple(color_value)
        PIXEL_MAP.update({tuple(position): color})

### system loop starts here
while supervisor.step(TIME_STEP) != -1:

    while receiver.getQueueLength() > 0:
        #receive bot attributes
        bot = None
        raw_data = receiver.getData()
        (id, message) = pickle.loads(raw_data)
        position = message.get("position")
        dsValues = message.get("dsValues")
        heading = message.get("heading")
        emitter.setChannel(id)
        
        #check if bot is registered
        if id in [bot.id for bot in bots]:
            #if server is no longer looking for new bots
            if server_state != ServerState.WAITING_FOR_CONNECTIONS:
                #send command
                bot = next(bot for bot in bots if bot.id == id)
                bot.set_position(np.array(position))
                bot.set_dsValues(dsValues)
                bot.set_heading(heading)
                bot.set_state(get_state(bot))

                cd.scan(bot) #collision detection/avoidance

                downlink_data = (bot.state, bot.color)
                send_message(downlink_data)
        #register new bot
        else:
            bot = Bot(id=id, position=np.array(position))
            bots.append(bot)
        #check for new messages
        receiver.nextPacket()
    
    #register bots
    if server_state == ServerState.WAITING_FOR_CONNECTIONS and len(bots) == GRID_SIZE**2:
        construct_grid()
        server_state = ServerState.CALCULATING_OPTIMAL_ASSIGNMENT
        calculate_optimal_assignment()

    #continue with next shell or start moving as swarm
    if current_shell_in_formation():
        if current_shell == MAX_SHELL:
            server_state = ServerState.MOVING_AS_SWARM

        if server_state == ServerState.WAITING_FOR_FORMATION:
            current_shell += 1

    #calculate next target of bots in swarm
    if server_state == ServerState.MOVING_AS_SWARM and all_bots_in_formation():
        offset = next(offset_generator)
        offset_all_bots(offset)

