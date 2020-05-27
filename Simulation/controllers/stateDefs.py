from enum import Enum, auto

class ServerState(Enum):
    WAITING_FOR_CONNECTIONS = auto()
    CALCULATING_OPTIMAL_ASSIGNMENT = auto()
    WAITING_FOR_FORMATION = auto()
    ASSIGNING_COLORS = auto()
    DONE = auto()

class BotState(Enum):
    IDLE = auto()
    TRAVELLING_NORTH = auto()
    TRAVELLING_EAST = auto()
    TRAVELLING_SOUTH = auto()
    TRAVELLING_WEST = auto()
    IN_FORMATION = auto()