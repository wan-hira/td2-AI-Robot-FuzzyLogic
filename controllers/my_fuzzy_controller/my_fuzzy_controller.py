"""my_fuzzy_controller controller.
TP2-AI-Webots and Fuzzy-Logic
Waneeza Hira AWAN - M1 Group 1
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#import Robot from controller 
from controller import Robot

# Maximum rotational speed of the e-puck wheels (in radians per second)
MAX_SPEED = 6.28
CRUISE_SPEED = 0.5 * MAX_SPEED

# Fonction to initialize the e-puck robot, motors and proximity distance sensors
def initialize_robot():
    #creating a instance of the robot
    robot = Robot()
    
    #Get the simulation time step in ms
    timestep = int(robot.getBasicTimeStep())
    
    #Initialize left and right wheel motors
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    
    #Set target position to infinity to enable continious velocity control
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float'"inf")
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    #Sensors names
    #Front-diagonal : ps1 (right), ps6 (left)
    #Side : ps2 (right), ps5 (left)
    #Rear : ps3 (right), ps4 (left)
    #Front : ps0 (right), ps7 (left)
    sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
    distance_sensors = {}
    
    for names in sensor_names:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        distance_sensors[name] = sensor
        
    return robot, timestep, left_motor, right_motor, distance_sensors
    
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
