"""bot_controller controller."""

from controller import Supervisor
import pickle
import random

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
        "N": move_north,
        "E": move_east,
        "S": move_south,
        "W": move_west,
        "I": stop_wheels
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
