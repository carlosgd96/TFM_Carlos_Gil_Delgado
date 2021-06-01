# Import the necessary libraries
from gripable.udpclient.flex.FlexUdp import FlexUdp
from gripable.udpclient.flex.FlexUdp2 import FlexUdp2
from gripable.udpclient.flex.protobuf.SensorData_pb2 import SensorData
import time
import RPi.GPIO as gpio
import statistics

# Start connections with the GripAble devices
flex_right = FlexUdp()
flex_left = FlexUdp2()
flex_right.start()
flex_left.start()
acc_min = 0.3        # Minimum acceleration which is detected by the GripAble device
goForward = True     # Variable to change between forward and reverse modes

# Previous accelerations to compute mean values
prev_acc_right = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
prev_acc_left = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

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
gpio.setup(16, gpio.OUT)
gpio.setup(33, gpio.OUT)
gpio.setup(37, gpio.OUT)

# Declaration of led's which notice the direction of the car
gpio.setup(24, gpio.OUT)  # Blue led for backward mode
gpio.output(24, gpio.LOW)
gpio.setup(40, gpio.OUT)  # Green led for forward mode
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
    if value > 1.0:
        value = 1.0
    if value < acc_min:
        value = 0
    return value * 100


# Function to spin in place moving to the right
def spin_in_place_right(velocity):
    motor_right_reverse.stop()
    motor_left_forward.stop()
    motor_right_forward.start(0)
    motor_left_reverse.start(0)
    motor_right_forward.ChangeDutyCycle(velocity)
    motor_left_reverse.ChangeDutyCycle(velocity)


# Function to spin in place moving to the left
def spin_in_place_left(velocity):
    motor_right_forward.stop()
    motor_left_reverse.stop()
    motor_right_reverse.start(0)
    motor_left_forward.start(0)
    motor_right_reverse.ChangeDutyCycle(velocity)
    motor_left_forward.ChangeDutyCycle(velocity)


# Function to read distance with ultrasonic sensor
def read_distance(trigger, echo):
    global end_time, start_time
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
    if (distance < 5) or (distance > 20):
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

        # Read the data of the GripAble devices
        devRdata: SensorData = flex_right.getSensorData()
        devLdata: SensorData = flex_left.getSensorData()

        # Saving acceleration and force values of both GripAble devices
        acc_right = float(devRdata.accelerometer.x)
        acc_right_abs = abs(acc_right)
        acc_left = float(devLdata.accelerometer.x)
        acc_left_abs = abs(acc_left)
        force_right = devRdata.grip.force
        force_left = devLdata.grip.force

        # Deleting first element of previous acceleration lists and save the new one
        del prev_acc_right[0]
        prev_acc_right.append(acc_right_abs)
        del prev_acc_left[0]
        prev_acc_left.append(acc_left_abs)

        # Computation of the mean values and limit both accelerations
        mean_acc_right = multi * statistics.mean(prev_acc_right)
        mean_acc_right = value_limits(mean_acc_right)
        print("Mean acc right: %f \n" % mean_acc_right, end="")
        mean_acc_left = multi * statistics.mean(prev_acc_left)
        mean_acc_left = value_limits(mean_acc_left)
        print("Mean acc left: %f \n" % mean_acc_left, end="")

        # Mode 1: forward and reverse, Mode 2: spin in place right direction, Mode 3: spin in place left direction
        mode = 1

        # If force is applied to both GripAble devices change between forward and reverse
        if (force_right > 1) and (force_left > 1):
            if goForward:
                gpio.output(40, gpio.LOW)  # Turn off the front led
                gpio.output(24, gpio.HIGH)  # Turn on the back led
                goForward = False
                print("Reverse mode \n")
            else:
                gpio.output(24, gpio.LOW)  # Turn off the back led
                gpio.output(40, gpio.HIGH)  # Turn on the front led
                goForward = True
                print("Forward mode \n")
            time.sleep(1)

        # While force is applied to the right GripAble it will spin in place to the right
        elif (force_right > 1) and (force_left < 1) and (mean_acc_right > acc_min):
            print("Spin in place mode to the right \n")
            mode = 2
            time.sleep(1)

        # While force is applied to the left GripAble it will spin in place to the left
        elif (force_right < 1) and (force_left > 1) and (mean_acc_left > acc_min):
            print("Spin in place mode to the left \n")
            mode = 3
            time.sleep(1)

        if mode == 1:  # Forward and reverse mode
            if (mean_acc_right > acc_min) or (mean_acc_left > acc_min):
                if goForward:  # Forward movement
                    # Apply same velocity to both wheels if accelerations are similar
                    if abs(mean_acc_right - mean_acc_left) < 40:
                        vel = max(mean_acc_right, mean_acc_left)
                        print("Same velocity %f \n" % vel, end="")
                        forward_right_motor(vel)
                        forward_left_motor(vel - 2)
                    else:
                        forward_right_motor(mean_acc_right)
                        forward_left_motor(mean_acc_left)

                else:  # Reverse movement
                    # Apply same velocity to both wheels if accelerations are similar
                    if abs(mean_acc_right - mean_acc_left) < 40:
                        vel = max(mean_acc_right, mean_acc_left)
                        reverse_right_motor(vel)
                        reverse_left_motor(vel - 2)
                    else:
                        reverse_right_motor(mean_acc_right)
                        reverse_left_motor(mean_acc_left)

            if mean_acc_right < acc_min:  # If right GripAble is not moving stop right motor
                motor_right_forward.stop()
                motor_right_reverse.stop()

            if mean_acc_left < acc_min:  # If left GripAble is not moving stop left motor
                motor_left_forward.stop()
                motor_left_reverse.stop()

        elif mode == 2:  # Spin in place to the right mode
            spin_in_place_right(mean_acc_right)

        elif mode == 3:  # Spin in place to the left mode
            spin_in_place_left(mean_acc_left)

    except KeyboardInterrupt:
        break

# Stop connections with the GripAble devices and clean the gpio configuration
print("MOVit movement program stopped")
gpio.cleanup()
flex_right.stop()
flex_right.join()
flex_left.stop()
flex_left.join()
