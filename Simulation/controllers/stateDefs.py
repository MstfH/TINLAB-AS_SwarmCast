from enum import Enum

class ServerState(Enum):
    WAITING_FOR_CONNECTIONS = 1
    CALCULATING_OPTIMAL_ASSIGNMENT = 2
    WAITING_FOR_FORMATION = 3
    DONE = 4

class BotState(Enum):
    IDLE = 'I'
    TRAVELLING_NORTH = 'N'
    TRAVELLING_EAST = 'E'
    TRAVELLING_SOUTH = 'S'
    TRAVELLING_WEST = 'W'
    IN_FORMATION = 'IF'
    TURNING_CW = 'CW'
    TURNING_CCW = 'CCW'