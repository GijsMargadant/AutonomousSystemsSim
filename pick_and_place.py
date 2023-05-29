"""Sample Webots controller for the pick and place benchmark."""

from controller import Robot
import math

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Basic robot dimentions (m)
wheelRadius = 0.05
wheelCircumference = math.pi * 2 * wheelRadius

# Inizialize base motors.
wheels = []
wheels.append(robot.getDevice("wheel1"))
wheels.append(robot.getDevice("wheel2"))
wheels.append(robot.getDevice("wheel3"))
wheels.append(robot.getDevice("wheel4"))
for wheel in wheels:
    # Activate controlling the motors setting the velocity.
    # Otherwise by default the motor expects to be controlled in force or position,
    # and setVelocity will set the maximum motor velocity instead of the target velocity.
    wheel.setPosition(float('+inf'))
    
# Initialize wheel position sensors.
# These sensors can be used to get the current wheel position and monitor the wheels movements.
wheelPositionSensors = []
wheelPositionSensors.append(robot.getDevice("wheel1sensor"))
wheelPositionSensors.append(robot.getDevice("wheel2sensor"))
wheelPositionSensors.append(robot.getDevice("wheel3sensor"))
wheelPositionSensors.append(robot.getDevice("wheel4sensor"))
for sensor in wheelPositionSensors:
    sensor.enable(timestep)

# Initialize arm motors.
armMotors1 = []
armMotors1.append(robot.getDevice("arm1"))
armMotors1.append(robot.getDevice("arm2"))
armMotors1.append(robot.getDevice("arm3"))
armMotors1.append(robot.getDevice("arm4"))
armMotors1.append(robot.getDevice("arm5"))
# Set the maximum motor velocity.
armMotors1[0].setVelocity(1.4)
armMotors1[1].setVelocity(1.4)
armMotors1[2].setVelocity(1.4)
armMotors1[3].setVelocity(1.4)

# Initialize arm motors.
armMotors2 = []
armMotors2.append(robot.getDevice("front arm1"))
armMotors2.append(robot.getDevice("front arm2"))
armMotors2.append(robot.getDevice("front arm3"))
armMotors2.append(robot.getDevice("front arm4"))
armMotors2.append(robot.getDevice("front arm5"))
# Set the maximum motor velocity.
armMotors2[1].setVelocity(1.4)
armMotors2[2].setVelocity(1.4)
armMotors2[3].setVelocity(1.4)
armMotors2[0].setVelocity(1.4)

# Initialize arm position sensors.
# These sensors can be used to get the current joint position and monitor the joint movements.
armPositionSensors1 = []
armPositionSensors1.append(robot.getDevice("arm1sensor"))
armPositionSensors1.append(robot.getDevice("arm2sensor"))
armPositionSensors1.append(robot.getDevice("arm3sensor"))
armPositionSensors1.append(robot.getDevice("arm4sensor"))
armPositionSensors1.append(robot.getDevice("arm5sensor"))
for sensor in armPositionSensors1:
    sensor.enable(timestep)
    
armPositionSensors2 = []
armPositionSensors2.append(robot.getDevice("front arm1sensor"))
armPositionSensors2.append(robot.getDevice("front arm2sensor"))
armPositionSensors2.append(robot.getDevice("front arm3sensor"))
armPositionSensors2.append(robot.getDevice("front arm4sensor"))
armPositionSensors2.append(robot.getDevice("front arm5sensor"))
for sensor in armPositionSensors2:
    sensor.enable(timestep)

# Initialize gripper motors.
finger11 = robot.getDevice("finger1")
finger12 = robot.getDevice("finger2")
finger21 = robot.getDevice("front finger1")
finger22 = robot.getDevice("front finger2")
# Get finger sensors
finger11Sensor = robot.getDevice("finger1sensor")
finger12Sensor = robot.getDevice("finger2sensor")
finger21Sensor = robot.getDevice("front finger1sensor")
finger22Sensor = robot.getDevice("front finger2sensor")
# Read the miminum and maximum position of the gripper motors.
fingerMinPosition = finger11.getMinPosition()
fingerMaxPosition = finger11.getMaxPosition()

def driveVehicle(distance, angle=0):
    maxVelocity = 14 # Radians per second
    
    # Determine scaling factor for each wheel velocity
    factor_rf_lb = math.sin(angle + math.pi/4)
    factor_lf_rb = math.cos(angle + math.pi/4)
    
    # The fastest moving wheel needs to rotate at maximum velocity
    velocity_rf_lb = maxVelocity * factor_rf_lb / max(abs(factor_rf_lb), abs(factor_lf_rb))
    velocity_lf_rb = maxVelocity * factor_lf_rb / max(abs(factor_rf_lb), abs(factor_lf_rb))
    
    # Set wheel velocity
    for i in range(0, len(wheels)):
        if i == 0 or i == 3:
            wheels[i].setVelocity(velocity_rf_lb)
        else:
            wheels[i].setVelocity(velocity_lf_rb)
            
    wheelPositions = []
    for wheelPositionSensor in wheelPositionSensors:
        if math.isnan(wheelPositionSensor.getValue()):
            wheelPositions.append(0)
        else:
            wheelPositions.append(wheelPositionSensor.getValue())
            
    totalMovement = 0
    while robot.step(timestep) != -1:
        # Determine the average movement of each wheel
        avgMovement = 0
        for i in range(0, len(wheelPositionSensors)):
            currentPosition = wheelPositionSensors[i].getValue()
            movement = abs(wheelPositions[i] - currentPosition) / (2 * math.pi) * wheelCircumference
            avgMovement += movement / len(wheels)
            wheelPositions[i] = currentPosition
        
        totalMovement += avgMovement
        
        print(f'avg dx: {avgMovement}, Total dx: {totalMovement}')
        
        if totalMovement >= distance:
            # Stop moving forward.
            for wheel in wheels:
                wheel.setVelocity(0.0)
            break
        
def closeGripper(finger1, finger2, finger1Sensor, finger2Sensor):
    finger1.setVelocity(0.1)
    finger2.setVelocity(0.1)
    
    finger1.setPosition(0.0)
    finger2.setPosition(0.0)
    
    finger1Sensor.enable(timestep)
    finger2Sensor.enable(timestep)
        
    while robot.step(timestep) != -1:
        prevFinger1Position = finger1Sensor.getValue()
        prevFinger2Position = finger2Sensor.getValue()
        print(f'dx: {(prevFinger1Position - prevFinger2Position)}')
        if abs(prevFinger1Position - prevFinger2Position) <= 0.049:
            finger1Sensor.disable()
            finger2Sensor.disable()
            break
            
        
def openGripper(finger1, finger2, finger1Sensor, finger2Sensor):
    finger1.setVelocity(0.2)
    finger2.setVelocity(0.2)
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)
    robot.step(5 * timestep)
        

# Move arm towards cube
armMotors1[1].setPosition(-0.5)
armMotors1[2].setPosition(-0.9)
armMotors1[3].setPosition(-1.25)

# Move forward.
# Distance to move: 2.912 m
driveVehicle(distance=2.912)


# Wait for 25 timesteps
openGripper(finger11, finger12, finger11Sensor, finger12Sensor)
openGripper(finger21, finger22, finger21Sensor, finger22Sensor)

robot.step(15 * timestep)

armMotors1[2].setPosition(-0.95)
robot.step(5 * timestep)

closeGripper(finger11, finger12, finger11Sensor, finger12Sensor)

# Wait for 25 timesteps
robot.step(25 * timestep)

armMotors1[1].setPosition(-0.3)

robot.step(25 * timestep)

armMotors1[1].setPosition(-0.95)
armMotors1[2].setPosition(1.8)
armMotors1[3].setPosition(1.0)
armMotors1[4].setPosition(math.pi / 2)

armMotors2[1].setPosition(-0.95)
armMotors2[2].setPosition(1.85)
armMotors2[3].setPosition(1.0)

driveVehicle(distance=1.0, angle=math.pi * 6 / 7 - 0.025)

closeGripper(finger21, finger22, finger21Sensor, finger22Sensor)

driveVehicle(distance=0.05, angle=math.pi * 6 / 7 - 0.025)

openGripper(finger11, finger12, finger11Sensor, finger12Sensor)

armMotors1[1].setPosition(-1.1)
armMotors1[2].setPosition(1.2)
armMotors2[3].setPosition(-1.0)

driveVehicle(distance=0.2, angle=math.pi * 6 / 7 - 0.025)

armMotors2[1].setPosition(0.0)
armMotors2[2].setPosition(-1.5)

driveVehicle(distance=0.93, angle=math.pi * 6 / 7 - 0.025)

robot.step(25 * timestep)

openGripper(finger21, finger22, finger21Sensor, finger22Sensor)
