"""bot_controller controller."""

from controller import Supervisor
import pickle
import random
import sys
sys.path.append('..')
from stateDefs import BotState as BotState

MOVE_FACTOR = 3
SPIN_FACTOR = 1
TIME_STEP = 32
ID = random.randint(1, 1000000)
direction = None
color = None

### webots node inits
supervisor = Supervisor()
robot_node = supervisor.getSelf()
translation_field = robot_node.getField("translation")

emitter = supervisor.getEmitter("emitter")
receiver = supervisor.getReceiver("receiver")
receiver.enable(TIME_STEP)
receiver.setChannel(ID)
compass = supervisor.getCompass("compass")
compass.enable(TIME_STEP)
led = supervisor.getLED("led")

wheels = [
    supervisor.getMotor("wheel1"),
    supervisor.getMotor("wheel2"),
    supervisor.getMotor("wheel3")
]

for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)

# populate list of distance sensors
ds = [supervisor.getDistanceSensor(f"ds{i}") for i in range(8)]
for sensor in ds: sensor.enable(TIME_STEP)

def readSensors():
    # reads sensors and returns list of sensor values
    raw_values = [sensor.getValue() for sensor in ds]
    return [-1 if value == 1000 else round(value, 2) for value in raw_values]


def send_message(message):
    emitter.send(pickle.dumps((ID, message)))

### wheel movements per command
def stop_wheels():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(0)


def move_north():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(-MOVE_FACTOR)
    wheels[2].setVelocity(MOVE_FACTOR)


def move_east():
    wheels[0].setVelocity(-2 * MOVE_FACTOR)
    wheels[1].setVelocity(MOVE_FACTOR)
    wheels[2].setVelocity(MOVE_FACTOR)


def move_south():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(MOVE_FACTOR)
    wheels[2].setVelocity(-MOVE_FACTOR)


def move_west():
    wheels[0].setVelocity(2 * MOVE_FACTOR)
    wheels[1].setVelocity(-MOVE_FACTOR)
    wheels[2].setVelocity(-MOVE_FACTOR)

def rotate_cw():
    wheels[0].setVelocity(1 * SPIN_FACTOR)
    wheels[1].setVelocity(1 * SPIN_FACTOR)
    wheels[2].setVelocity(1 * SPIN_FACTOR)

def rotate_ccw():
    wheels[0].setVelocity(-1 * SPIN_FACTOR)
    wheels[1].setVelocity(-1 * SPIN_FACTOR)
    wheels[2].setVelocity(-1 * SPIN_FACTOR)


def set_color(color):
    led.set(int(color, 16))

### commands per state
def move(direction):
    move_map = {
        BotState.IDLE: stop_wheels,
        BotState.TRAVELLING_NORTH: move_north,
        BotState.TRAVELLING_EAST: move_east,
        BotState.TRAVELLING_SOUTH: move_south,
        BotState.TRAVELLING_WEST: move_west,
        BotState.IN_FORMATION: stop_wheels,
        BotState.EMERGENCY_BRAKE: stop_wheels,
        BotState.TURNING_CW: rotate_cw,
        BotState.TURNING_CCW: rotate_ccw
    }
    func = move_map.get(direction)
    func()

while supervisor.step(TIME_STEP) != -1:

    #print ID when selected in webots simulation
    if supervisor.getSelected() and supervisor.getSelected().getId() == supervisor.getSelf().getId():
        print(f"Selected: <{ID}> {direction}")

    #receive command
    while receiver.getQueueLength() > 0:
        #parse command
        raw_data = receiver.getData()
        direction, color = pickle.loads(raw_data)
        move(direction)
        set_color(color)
        receiver.nextPacket()
    #read attributes
    dsValues = readSensors()
    current_position = translation_field.getSFVec3f()
    (x, _, z) = current_position
    (_,_,heading) = compass.getValues()
    heading = round(heading,5)
    x = round(x,3)
    z = round(z,3)
    #send attributes to server
    message = {
        "position" : [x, z],
        "dsValues" : dsValues,
        "heading" : heading
    }
    send_message(message)
