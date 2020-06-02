import sys

sys.path.append('..')
from stateDefs import ServerState as ServerState
from stateDefs import BotState as BotState

class Bot:

    def __init__(self, id, position):
        print(f"Registered bot {id} @ {position}")
        self.id = id
        self.position = position
        self.target = None
        self.shell = None
        self.state = BotState.IDLE

    def set_position(self, position):
        self.position = position

    def set_target(self, target):
        self.target = target

    def set_shell(self, shell):
        self.shell = shell

    def set_state(self, state):
        self.state = state