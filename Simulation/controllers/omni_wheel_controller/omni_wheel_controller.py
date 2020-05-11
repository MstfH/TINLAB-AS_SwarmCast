"""omni_wheels controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
MAX_SPEED = 6.28



###Setup###
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
#init wheels and put in list
wheels = []
wheelsNames = ['wheel1','wheel2','wheel3']
for name in wheelsNames:
    wheels.append(robot.getMotor(name))
#set wheel params
for wheelNum in range(3):
    wheels[wheelNum].setPosition(float('inf'))
    wheels[wheelNum].setVelocity(0.0)
# You should insert a getDevic-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

###Functions###
def moveEast(velocity, pos):
    wheels[0].setPosition(pos * -1.0)
    wheels[1].setPosition(pos * 0.5)
    wheels[2].setPosition(pos * 0.5)
    
    wheels[0].setVelocity(velocity*2)
    wheels[1].setVelocity(velocity*1)
    wheels[2].setVelocity(velocity*1)
    
def moveSouth(velocity, pos):
    # wheels[0].setVelocity(0.0)
    # wheels[1].setVelocity(1.0)
    # wheels[2].setVelocity(-1.0)
    wheels[0].setPosition(pos * 0.0)
    wheels[1].setPosition(pos * 0.5)
    wheels[2].setPosition(pos * -0.5)
    
    wheels[0].setVelocity(velocity*1)
    wheels[1].setVelocity(velocity*1)
    wheels[2].setVelocity(velocity*1)    
    

def moveWest(velocity, pos):
    # wheels[0].setVelocity(2.0)
    # wheels[1].setVelocity(-1.0)
    # wheels[2].setVelocity(-1.0)
    wheels[0].setPosition(pos * 1.0)
    wheels[1].setPosition(pos * -0.5)
    wheels[2].setPosition(pos * -0.5)
    
    wheels[0].setVelocity(velocity*2)
    wheels[1].setVelocity(velocity*1)
    wheels[2].setVelocity(velocity*1)    
        
def moveNorth(velocity, pos):
    # wheels[0].setVelocity(0.0)
    # wheels[1].setVelocity(-1.0)
    # wheels[2].setVelocity(1.0)
    wheels[0].setPosition(pos * 0.0)
    wheels[1].setPosition(pos * -0.5)
    wheels[2].setPosition(pos * 0.5)
    
    wheels[0].setVelocity(velocity*1)
    wheels[1].setVelocity(velocity*1)
    wheels[2].setVelocity(velocity*1)    

    

#### Main loop: ###
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    moveEast(2, 10.0)
    moveNorth(2, 10.0)
    moveWest(2, 10.0)
    moveSouth(2, 10.0)
    pass

# Enter here exit cleanup code.
