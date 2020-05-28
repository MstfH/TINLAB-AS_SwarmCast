"""bot_controller controller."""

from controller import Supervisor
import pickle
import random
import sys
sys.path.append('..')
from stateDefs import BotState as BotState

SPEED_FACTOR = 3
TIME_STEP = 32
ID = random.randint(1, 10000)

supervisor = Supervisor()
robot_node = supervisor.getSelf()
translation_field = robot_node.getField("translation")

emitter = supervisor.getEmitter("emitter")
receiver = supervisor.getReceiver("receiver")
receiver.enable(100)
receiver.setChannel(ID)

wheels = [
    supervisor.getMotor("wheel1"),
    supervisor.getMotor("wheel2"),
    supervisor.getMotor("wheel3")
]

for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)


def send_message(message):
    emitter.send(pickle.dumps((ID, message)))


def stop_wheels():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(0)


def move_north():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(-SPEED_FACTOR)
    wheels[2].setVelocity(SPEED_FACTOR)


def move_east():
    wheels[0].setVelocity(-2 * SPEED_FACTOR)
    wheels[1].setVelocity(SPEED_FACTOR)
    wheels[2].setVelocity(SPEED_FACTOR)


def move_south():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(SPEED_FACTOR)
    wheels[2].setVelocity(-SPEED_FACTOR)


def move_west():
    wheels[0].setVelocity(2 * SPEED_FACTOR)
    wheels[1].setVelocity(-SPEED_FACTOR)
    wheels[2].setVelocity(-SPEED_FACTOR)

def move(direction):
    move_map = {
        BotState.IDLE: stop_wheels,
        BotState.TRAVELLING_NORTH: move_north,
        BotState.TRAVELLING_EAST: move_east,
        BotState.TRAVELLING_SOUTH: move_south,
        BotState.TRAVELLING_WEST: move_west,
        BotState.IN_FORMATION: stop_wheels
    }
    func = move_map.get(direction)
    func()

while supervisor.step(TIME_STEP) != -1:
    while receiver.getQueueLength() > 0:
        raw_data = receiver.getData()
        direction = pickle.loads(raw_data)
        move(direction)
        receiver.nextPacket()

    current_position = translation_field.getSFVec3f()
    (x, _, z) = current_position
    send_message([x, z])
