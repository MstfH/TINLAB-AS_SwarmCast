from controller import Robot
import numpy as np


import sys
sys.path.append('..')
from stateDefs import ServerState as ServerState
from stateDefs import BotState as BotState


collision_queue = {}
proximityLimit = 100

#sensorKey=[BotState.TRAVELLING_EAST,5,6,7,0,1,2,3]
movementKey={
    #sensor(s) : direction when tripped
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
    tuple([7, 0]) :    BotState.TRAVELLING_EAST
}

def avoidCollision(bot, collisionAngles):
    #collisionAngles=repr(collisionAngles)
    movement = movementKey.get(collisionAngles)
    bot.update({
        "state": movement
    })
    print(bot)

def parse(dsValues):
    '''
    returns tuple with indices of tripped sensors, corrisponding to sensor number
    (index 0 is ds0)
    '''
    boolArray = np.zeros(8, dtype=bool)
    for i in dsValues:
        if (i != -1 and i < proximityLimit):
            boolArray[dsValues.index(i)] = True
    trippedSensors = np.nonzero(boolArray)
    return trippedSensors[0] #first element only needed
    
def scan(bot):
    '''
    scans the distance sensor values, stopping the bot if
    proximity limit is exceeded. Adds to collision queue.
    '''
    collisionAngles = parse(bot.get("dsValues"))
    ID = bot.get("id")
    if(np.size(collisionAngles) > 0):
        collisionAngles=tuple(collisionAngles)

        if bot.get('state') == BotState.IN_FORMATION:
            print(ID, 'in formation. Ignoring collision')
            if ID in collision_queue:
                collision_queue.pop(ID)
        else:
            print("Proximity Warning bot:", ID, collisionAngles)
            bot.update({
                "stateBeforeCollision": bot.get("state"),
                "state": BotState.EMERGENCY_BRAKE,
                "collision": collisionAngles
            })

            collision_queue.update({ID : collisionAngles})

            avoidCollision(bot, collisionAngles)

            # if max(collision_queue, key=int) == ID:
            #     print(ID, "higher priority, avoiding")
            #     avoidCollision(bot, collisionAngles)
            #     bot.update({
            #                 "stateBeforeCollision": None,
            #                 "collision": None
            #     })

        
        # if len(collision_queue) > 1:
        #     if (max(collision_queue, key=int) == ID):
        #         print(ID, "higher priority, continuing")
        #         bot.update({
        #             "state": BotState.BRAKE_RELEASED,
        #             "collision": None
        #         })
        #         collision_queue.pop(max(collision_queue))
    return