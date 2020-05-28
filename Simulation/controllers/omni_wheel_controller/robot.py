from controller import Robot, LED, Emitter, Receiver

# Here we are essentially just initializing the robot, its sensors and actuators
SPEED_FACTOR = 3
robot = Robot()
timestep = int(robot.getBasicTimeStep())

led = robot.getLED("led")

accelerometer = robot.getAccelerometer("accelerometer")
accelerometer.enable(timestep)

emitter = robot.getEmitter("transmitter")
receiver = robot.getReceiver("receiver")
receiver.enable(100)  # sampling period

# Create an iterable list of the motors/wheels
wheels = [
    robot.getMotor("wheel1"),
    robot.getMotor("wheel2"),
    robot.getMotor("wheel3")
]

for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)

# Create an iterable list of the distance sensors
ds = []
dsNames = [
    'ds0', 'ds1', 'ds2', 'ds3',
    'ds4', 'ds5', 'ds6', 'ds7'
]

for i in range(8):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(timestep)


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
        "I": stop_wheels,
        "N": move_north,
        "E": move_east,
        "S": move_south,
        "W": move_west,
        "IF": stop_wheels
    }
    func = move_map.get(direction)
    func()


step = robot.step
