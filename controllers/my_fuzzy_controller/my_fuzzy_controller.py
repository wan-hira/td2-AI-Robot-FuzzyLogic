"""my_fuzzy_controller controller.
TP2-AI-Webots and Fuzzy-Logic
Waneeza Hira AWAN - M1 Group 1
"""

#import Robot from controller 
from controller import Robot
from fuzzy_logic import compute_fuzzy_control

# Maximum rotational speed of the e-puck wheels (in radians per second)
MAX_SPEED = 6.28
CRUISE_SPEED = 0.5 * MAX_SPEED
# Distance between left and right wheels of the e-puck (in meters)
WHEEL_BASE = 0.052

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
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    #Sensors names mapping
    #Front-diagonal : ps1 (right), ps6 (left)
    #Side : ps2 (right), ps5 (left)
    #Rear : ps3 (right), ps4 (left)
    #Front : ps0 (right), ps7 (left)
    sensor_names = [ "ps1", "ps2", "ps3", "ps4", "ps5", "ps6"]
    distance_sensors = {}
    
    for name in sensor_names:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        distance_sensors[name] = sensor
        
    return robot, timestep, left_motor, right_motor, distance_sensors
    
def main():
    robot, timestep, left_motor, right_motor, distance_sensors = ( initialize_robot() )
    
    print("Controller started")
    
    #Main control loop
    while robot.step(timestep) != -1:
        #Read raw proximity values(range in 0 to 4000)
        #Higher value means an obstacle is closer to the sensor
        s = { 
            name : sensor.getValue() for name, sensor in distance_sensors.items() 
        }
        
        # Compute fuzzy outputs: normalized linear speed and steering angle
        speed_ratio, delta_theta = compute_fuzzy_control(
            left_front_diag=s["ps6"],
            left_side=s["ps5"],
            right_front_diag=s["ps1"],
            right_side=s["ps2"],
        )

        # Convert (v, delta_theta) to wheel speeds
        # Positive delta_theta turns LEFT (slows left wheel, speeds up right wheel)
        base_speed = speed_ratio * MAX_SPEED
        turn_diff = delta_theta * (MAX_SPEED * 0.5)

        left_speed = max(-MAX_SPEED, min(MAX_SPEED, base_speed - turn_diff))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, base_speed + turn_diff))
        #Move forward at cruise speed
        left_motor.setVelocity(CRUISE_SPEED)
        right_motor.setVelocity(CRUISE_SPEED)

if __name__ == "__main__":
    main()    

