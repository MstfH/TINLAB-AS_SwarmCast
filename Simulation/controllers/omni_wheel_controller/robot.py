from controller import Robot, LED, Lidar

# Here we are essentially just initializing the robot, its sensors and actuators
robot = Robot()
timestep = int(robot.getBasicTimeStep())

lidar = robot.getLidar("Velodyne Puck")
lidar.enable(timestep)
lidar.enablePointCloud()

led = robot.getLED("led")

accelerometer = robot.getAccelerometer("accelerometer")
accelerometer.enable(timestep)

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

step = robot.step
