# Import the necessary libraries
from gripable.udpclient.flex.FlexUdp import FlexUdp
from gripable.udpclient.flex.protobuf.SensorData_pb2 import SensorData
import time
import struct
import RPi.GPIO as gpio
import sys
import statistics

# Start connections with the GripAble device
flex = FlexUdp()
flex.start()
acc_min = 0.2  # Minimum acceleration which is detected by the GripAble device

# Previous accelerations to compute mean values
prev_acc_x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
prev_acc_y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

print("Initializing connection")
gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

# Variables for trigger and echo pins of ultrasonic sensors
Trigger_right = 29
Echo_right = 36
Trigger_left = 31
Echo_left = 32

# Declaration of driver pins
gpio.setup(12, gpio.OUT)
gpio.setup(33, gpio.OUT)
gpio.setup(16, gpio.OUT)
gpio.setup(37, gpio.OUT)

# Declaration of led's which notice the direction of the car
gpio.setup(24, gpio.OUT)  # Blue led for backward mode
gpio.setup(40, gpio.OUT)  # Green led for forward mode
gpio.output(24, gpio.LOW)
gpio.output(40, gpio.HIGH)

# Declaration of trigger and echo pins of ultrasonic sensors
gpio.setup(Trigger_right, gpio.OUT)
gpio.setup(Echo_right, gpio.IN)
gpio.output(Trigger_right, False)  # Setting the right trigger pin to zero as default
gpio.setup(Trigger_left, gpio.OUT)
gpio.setup(Echo_left, gpio.IN)
gpio.output(Trigger_left, False)  # Setting the left trigger pin to zero as default

# Buzzer to give sound feedback
gpio.setup(18, gpio.OUT)
beep = gpio.PWM(18, 100)
beep.start(0)

# Assignation of pins to motors
motor_right_forward = gpio.PWM(33, 100)
motor_right_reverse = gpio.PWM(37, 100)
motor_left_forward = gpio.PWM(12, 100)
motor_left_reverse = gpio.PWM(16, 100)


# Function to move forward right motor
def forward_right_motor(velocity):
    motor_right_reverse.stop()
    motor_right_forward.start(0)
    motor_right_forward.ChangeDutyCycle(velocity)


# Function to move reverse right motor
def reverse_right_motor(velocity):
    motor_right_forward.stop()
    motor_right_reverse.start(0)
    motor_right_reverse.ChangeDutyCycle(velocity)


# Function to move forward left motor
def forward_left_motor(velocity):
    motor_left_reverse.stop()
    motor_left_forward.start(0)
    motor_left_forward.ChangeDutyCycle(velocity)


# Function to move reverse left motor
def reverse_left_motor(velocity):
    motor_left_forward.stop()
    motor_left_reverse.start(0)
    motor_left_reverse.ChangeDutyCycle(velocity)


# Function to limit values between min_acc and 1
def value_limits(value):
    if abs(value) < acc_min:
        value = 0
    else:
        if value > 0:
            value = value + 0.2
            if value > 1:
                value = 1
        else:
            value = value - 0.2
            if value < -1.0:
                value = -1
    return value * 100


# Function to read distance with ultrasonic sensor
def read_distance(trigger, echo):
    gpio.output(trigger, True)
    time.sleep(0.00001)
    gpio.output(trigger, False)
    while gpio.input(echo) == 0:
        start_time = time.time()
    while gpio.input(echo) == 1:
        end_time = time.time()
    difference = end_time - start_time
    distance = 17150 * difference
    print("Measured distance is ", distance, " cm")

    # Use the buzzer to notice if the distance with the wall is very low or high
    if (distance < 5) or (distance > 30):
        beep.ChangeDutyCycle(100)
        time.sleep(0.01)
        beep.ChangeDutyCycle(0)


# Ask user to use the sensors of the right side, left one or no one
sensor = input("Please write 1 to use the right distance sensor, 2 to use the left one or 3 do not use them:   ")
sensor = float(sensor)

# Main loop
while True:
    try:
        time.sleep(0.05)  # Sleep time necessary to read measures of the GripAble device

        multi = 2
        if sensor == 1:  # The user chose use the right ultrasonic sensor
            read_distance(Trigger_right, Echo_right)
            multi = 1

        elif sensor == 2:  # The user chose use the left ultrasonic sensor
            read_distance(Trigger_left, Echo_left)
            multi = 1

        # Read the data of the GripAble device
        devdata: SensorData = flex.getSensorData()

        # Saving x and y accelerations of GripAble
        acc_x = float(devdata.accelerometer.x)
        acc_y = float(devdata.accelerometer.y)

        # Deleting first element of previous acceleration lists and save the new one
        del prev_acc_x[0]
        prev_acc_x.append(acc_x)
        del prev_acc_y[0]
        prev_acc_y.append(acc_y)

        # Computation of the mean values and limit both accelerations
        mean_acc_x = multi * statistics.mean(prev_acc_x)
        mean_acc_x = value_limits(mean_acc_x)
        print("Mean acc x: %f \n" % mean_acc_x, end="")
        mean_acc_y = multi * statistics.mean(prev_acc_y)
        mean_acc_y = value_limits(mean_acc_y)
        print("Mean acc y: %f \n" % mean_acc_y, end="")

        if mean_acc_x <= -acc_min:      # The user is bending forward the GripAble
            gpio.output(24, gpio.LOW)   # Turn off the back led
            gpio.output(40, gpio.HIGH)  # Turn on the front led

            # Move forward the robot
            forward_right_motor(-mean_acc_x * 0.75 - mean_acc_y * 0.25)
            forward_left_motor(-mean_acc_x * 0.70 + mean_acc_y * 0.25)

        elif mean_acc_x > acc_min:      # The user is bending backward the GripAble
            gpio.output(40, gpio.LOW)   # Turn off the front led
            gpio.output(24, gpio.HIGH)  # Turn on the back led

            # Move backward the robot
            reverse_right_motor(mean_acc_x * 0.75 - mean_acc_y * 0.25)
            reverse_left_motor(mean_acc_x * 0.70 + mean_acc_y * 0.25)

        elif mean_acc_x == 0 and abs(mean_acc_y) > acc_min: # The user is bending to the sideways the GripAble
            # Spin in place the robot
            if mean_acc_y < 0:          # Spin to the left
                forward_right_motor(-mean_acc_y)
                reverse_left_motor(-mean_acc_y)
            else:                       # Spin to the right
                reverse_right_motor(mean_acc_y)
                forward_left_motor(mean_acc_y)

        else:  # The user is not bending the GripAble device
            motor_right_forward.ChangeDutyCycle(0)
            motor_right_forward.stop()
            motor_right_reverse.ChangeDutyCycle(0)
            motor_right_reverse.stop()
            motor_left_forward.ChangeDutyCycle(0)
            motor_left_forward.stop()
            motor_left_reverse.ChangeDutyCycle(0)
            motor_left_reverse.stop()

    except KeyboardInterrupt:
        break

# Stop connections with the GripAble device and clean the gpio configuration
print("Joystick movement program stopped")
gpio.cleanup()
flex.stop()
flex.join()
