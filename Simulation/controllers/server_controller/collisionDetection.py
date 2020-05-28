from controller import Robot
import numpy as np


import sys
sys.path.append('..')
from stateDefs import ServerState as ServerState
from stateDefs import BotState as BotState


collision_queue = {}
proximityLimit = 100


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
        collisionAngles=list(collisionAngles)
        print("Proximity Warning bot:", ID, collisionAngles)
        # bot.update({
        #     "state": 'I',
        #     "collision": collisionAngles
        # })
        if bot.get("state") == BotState.IN_FORMATION:
            print(ID, )

        bot.update({
            "stateBeforeCollision": bot.get("state"),
            "state": BotState.EMERGENCY_BRAKE,
            "collision": collisionAngles
        })

        if ID not in collision_queue:
            collision_queue.update({ID : collisionAngles})
        elif ID in collision_queue: #updates if another sensor trips
            collision_queue.update({ID : collisionAngles})
        
        if len(collision_queue) > 1:
            if (max(collision_queue, key=int) == ID):
                print(ID, "higher priority, continuing")
                bot.update({
                    "state": BotState.BRAKE_RELEASED,
                    "collision": None
                })
                collision_queue.pop(max(collision_queue))
    return