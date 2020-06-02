"""sever_controller controller."""

from controller import Robot
import collisionDetection as cd
import pickle
from math import sqrt
import numpy as np
from scipy.optimize import linear_sum_assignment
import sys
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

server_state = ServerState.WAITING_FOR_CONNECTIONS
current_shell = 0

bots = []
bot_ids = []
robot = Robot()
emitter = robot.getEmitter("emitter")
receiver = robot.getReceiver("receiver")
receiver.enable(TIME_STEP)


def distance_between(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)


def get_shell(point):
    x, y = point
    center = GRID_ORIGIN + (GRID_SIZE - 1) / 2 * GRID_SPACING
    delta_x = abs(center - x)
    delta_y = abs(center - y)
    max_delta = max(delta_x, delta_y)
    steps = max_delta / GRID_SPACING
    return steps


def calculate_optimal_assignment():
    global server_state
    cost = np.empty([len(bots), len(bots)])
    for i in range(len(bots)):
        for j in range(len(bots)):
            cost[i, j] = np.sum(np.absolute(
                bots[i].get("position") - GRID_POSITIONS[j]))

    rows, cols = linear_sum_assignment(cost)
    for (row_i, col_i) in zip(rows, cols):
        bots[row_i].update({
            "target": GRID_POSITIONS[col_i],
            "shell": get_shell(GRID_POSITIONS[col_i])
        })
    server_state = ServerState.WAITING_FOR_FORMATION


def send_message(message):
    emitter.send(pickle.dumps(message))


def swap(bot1, bot2):
    target = bot1.get("target")
    state = bot1.get("state")
    shell = bot1.get("shell")

    bot1.update({
        "target": bot2.get("target"),
        "state": bot2.get("state"),
        "shell": bot2.get("shell"),
        "swapped": True
    })

    bot2.update({
        "target": target,
        "state": state,
        "shell": shell,
        "swapped": True
    })


def get_state(bot):
    id = bot.get("id")
    position = bot.get("position")
    target = bot.get("target")
    shell = bot.get("shell")
    pos_x, pos_y = position
    target_x, target_y = target

    if shell > current_shell:
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
    bots_in_current_shell = [
        bot for bot in bots if bot.get("shell") == current_shell]
    return all(bot.get("state") == BotState.IN_FORMATION for bot in bots_in_current_shell)


if GRID_SIZE % 2 == 0:
    print("WARNING: Please set GRID_SIZE to an uneven number.")
   
while robot.step(TIME_STEP) != -1:
    if len(bots) > GRID_SIZE**2:
        print("WARNING: Excess of bots.")

    while receiver.getQueueLength() > 0:
        bot = None
        raw_data = receiver.getData()
        (id, position) = pickle.loads(raw_data)
        #print(pickle.loads(raw_data))
        (id, message) = pickle.loads(raw_data)
        position = message.get("position")
        dsValues = message.get("dsValues")
        emitter.setChannel(id)

        if id in bot_ids:
            bot = next(bot for bot in bots if bot.get("id") == id)
            bot.update({
                "position": position,
                "dsValues": dsValues,
                "state": get_state(bot)
            })
            #TODO check from rebase
            # bot.update({"position": position})
            # new_state = get_state(
            #     bot.get("position"), bot.get("target")
            # )

            cd.scan(bot)

            #if id in cd.collision_queue:


            # print("a", bot)
            # if bot.get("state") == BRAKE_RELEASED:
            #     bot.update({
            #         "state": bot.get("stateBeforeCollision"),
            #         "stateBeforeCollision": None
            #     })
            #     print("b", bot)
        else:
            print(f"Registered bot {id} @ {position}")
            bots.append({
                "id": id,
                "position": np.array(position),
                "target": None,
                "state": BotState.IDLE,
                "dsValues": None,
                "collision": None,
                "stateBeforeCollision": None
            })
            bot_ids.append(id)
            bot = bots[-1]
        
        send_message(bot.get("state"))

        if server_state == ServerState.WAITING_FOR_CONNECTIONS and len(bots) == GRID_SIZE**2:
            server_state = ServerState.CALCULATING_OPTIMAL_ASSIGNMENT
            calculate_optimal_assignment()

        if current_shell_in_formation():
            if current_shell == MAX_SHELL:
                server_state = ServerState.ASSIGNING_COLORS

            if server_state == ServerState.WAITING_FOR_FORMATION:
                current_shell += 1

        receiver.nextPacket()
