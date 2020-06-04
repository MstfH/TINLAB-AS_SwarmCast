from enum import Enum

class ServerState(Enum):
    WAITING_FOR_CONNECTIONS = 1
    CALCULATING_OPTIMAL_ASSIGNMENT = 2
    WAITING_FOR_FORMATION = 3
    DONE = 4

class BotState(Enum):
    IDLE = 'I'
    TRAVELLING_NORTH = 0 #'N'
    TRAVELLING_EAST = 2 #'E'
    TRAVELLING_SOUTH = 4 #'S'
    TRAVELLING_WEST = 6 #'W'
    IN_FORMATION = 'IF'
    EMERGENCY_BRAKE = 'EB'