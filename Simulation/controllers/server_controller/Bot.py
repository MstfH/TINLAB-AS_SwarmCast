import sys

from collections import deque

sys.path.append('..')
from stateDefs import ServerState as ServerState
from stateDefs import BotState as BotState

SWAPS_BEFORE_RESET = 2 #how many total swaps can be made before swap between oldest ID's is allowed

class Bot:
    def __init__(self, id, position):
        print(f"Registered bot {id} @ {position}")
        self.id = id
        self.position = position
        self.target = None
        self.shell = None
        self.swapdeque = deque(maxlen = SWAPS_BEFORE_RESET)
        self.state = BotState.IDLE
        self.heading = 0
        self.color = "0x000000"
        self.dsValues = []
        self.stateBeforeCollision = BotState.IDLE
        self.collision = None

    def set_position(self, position): 
        self.position = position

    def set_target(self, target):
        self.target = target

    def set_shell(self, shell):
        self.shell = shell
    
    def append_swapdeque(self, bot):
        self.swapdeque.append(bot)

    def set_state(self, state):
        self.state = state

    def set_heading(self, heading):
        self.heading = heading
        
    def set_color(self, color):
        self.color = color
    
    def set_dsValues(self, dsValues):
        self.dsValues = dsValues
    
    def set_stateBeforeCollision(self, stateBeforeCollision):
        self.stateBeforeCollision = stateBeforeCollision
    
    def set_collision(self, collision):
        self.collision = collision
