import sys
sys.path.append('..')
from Bot import Bot
from controller import Robot
from stateDefs import ServerState as ServerState
from stateDefs import BotState as BotState
import numpy as np

collision_queue = {}
PROXIMITY_LIMIT = 100

'''
sensor(s)   :   direction to travel when tripped
                (always left)
'''
movementKey={
    tuple([0]) :       BotState.TRAVELLING_WEST,
    tuple([0, 1]) :    BotState.TRAVELLING_WEST,
    tuple([1]) :       BotState.TRAVELLING_WEST,
    tuple([1, 2]) :    BotState.TRAVELLING_SOUTH,
    tuple([2]) :       BotState.TRAVELLING_NORTH,
    tuple([2, 3]) :    BotState.TRAVELLING_NORTH,
    tuple([3]) :       BotState.TRAVELLING_NORTH,
    tuple([3, 4]) :    BotState.TRAVELLING_WEST,
    tuple([4]) :       BotState.TRAVELLING_EAST,
    tuple([4, 5]) :    BotState.TRAVELLING_EAST,
    tuple([5]) :       BotState.TRAVELLING_EAST,
    tuple([5, 6]) :    BotState.TRAVELLING_EAST,
    tuple([6]) :       BotState.TRAVELLING_SOUTH,
    tuple([6, 7]) :    BotState.TRAVELLING_SOUTH,
    tuple([7]) :       BotState.TRAVELLING_SOUTH,
    tuple([0, 7]) :    BotState.TRAVELLING_EAST
}

def avoidCollision(bot, collisionAngles):
    '''
    Moves robot left of collision using the movementKey as decision tree
    '''
    movement = movementKey.get(collisionAngles)
    if movement == None:
        movement = BotState.IDLE
    bot.set_state(movement)

def parse(dsValues):
    '''
    returns tuple with indices of tripped sensors, corrisponding to sensor number
    (index 0 is ds0)
    '''
    boolArray = np.zeros(8, dtype=bool)
    for i in dsValues:
        if (i != -1 and i < PROXIMITY_LIMIT):
            boolArray[dsValues.index(i)] = True
    trippedSensors = np.nonzero(boolArray)
    return trippedSensors[0] #first element only needed

def logCollision(bot, collisionAngles):
    '''
    Saves the state before collision, activates emergency brake
    and logs the angle(s) of collision
    '''
    bot.set_stateBeforeCollision(bot.state)
    bot.set_state(BotState.EMERGENCY_BRAKE)
    bot.set_collision(collisionAngles)
    
def scan(bot):
    '''
    scans the distance sensor values, stopping the bot if
    proximity limit is exceeded. Adds to collision queue.
    '''
    collisionAngles = parse(bot.dsValues)
    ID = bot.id
    if(np.size(collisionAngles) > 0):
        collisionAngles=tuple(collisionAngles)

        if bot.state == BotState.IN_FORMATION:
            print(ID, 'in formation. Ignoring collision')
            if ID in collision_queue:
                collision_queue.pop(ID)
        else:
            print("Proximity Warning bot:", ID, collisionAngles)

            logCollision(bot, collisionAngles)
            collision_queue.update({ID : collisionAngles})

            avoidCollision(bot, collisionAngles)
    return