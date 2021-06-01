# Import the necessary libraries
from gripable.udpclient.flex.FlexUdp import FlexUdp
from gripable.udpclient.flex.FlexUdp2 import FlexUdp2
from gripable.udpclient.flex.protobuf.SensorData_pb2 import SensorData
import time
import RPi.GPIO as gpio
import statistics

# Start connections with the GripAble device
flex = FlexUdp()
flex.start()
acc_min = 0.2  # Minimum acceleration which is detected by the GripAble device

ref_Speed = 65.5  # Ref voltage 6 V = no speed
ref_Direction = 66.5  # Ref voltage 6 V = no turn

# Previous accelerations to compute mean values
prev_acc_right = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
prev_acc_left = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

print("Initializing connection")
gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

# Declaration of driver pins
gpio.setup(11, gpio.OUT)
gpio.setup(15, gpio.OUT)
gpio.setup(16, gpio.OUT)
gpio.setup(31, gpio.OUT)

# Assignation of pins to motors
gpio.setup(11, gpio.HIGH)  # Enable signal speed of the motors
gpio.setup(15, gpio.HIGH)  # Enable signal direction of the motors
motor_forward_reverse = gpio.PWM(16, 20)
motor_forward_reverse.start(0)
motor_forward_reverse.ChangeDutyCycle(ref_Speed)
motor_right_left = gpio.PWM(31, 20)
motor_right_left.start(0)
motor_right_left.ChangeDutyCycle(ref_Direction)


# Function to amplify and limit values between min_acc and 1
# The values are amplified due to GripAble will not be in the maximum inclination
def value_limits(value):
    if abs(value) < acc_min:
        value = 0
    else:
        if value > 0:
            if value > 0.8:
                value = 0.8
        else:
            if value < -0.8:
                value = -0.8
    return value * 100


def move_forward_reverse(speed):
    # The ref_Direction is 66, the maximum value is 80 and the minimum 52.5, the 0.175 is the ratio to multiply
    # being 14 the maximum adding or reduction due to the range of motion is [20, 100]
    speed = speed * 0.175
    if goForward:
        motor_forward_reverse.ChangeDutyCycle(ref_Speed + speed)
    else:
        motor_forward_reverse.ChangeDutyCycle(ref_Speed - speed)


def move_right_left(direction):
    # The ref_Direction is 66, the maximum value is 80 and the minimum 52.5, the 0.175 is the ratio to multiply
    # being 14 the maximum adding or reduction due to the range of motion is [20, 100]
    direction = direction * 0.175
    if direction > acc_min:
        motor_right_left.ChangeDutyCycle(ref_Direction + direction)
    elif direction < acc_min:
        motor_right_left.ChangeDutyCycle(ref_Direction - direction)
    else:
        motor_right_left.ChangeDutyCycle(ref_Direction)


def spin_in_place(direction):
    # The ref_Direction is 66, the maximum value is 80 and the minimum 52.5, the 0.175 is the ratio to multiply
    # being 14 the maximum adding or reduction due to the range of motion is [20, 100]
    direction = direction * 0.175
    if mode == 2:
        motor_right_left.ChangeDutyCycle(ref_Direction + direction)
    else:
        motor_right_left.ChangeDutyCycle(ref_Direction - direction)


# Main loop
while True:
    try:
        time.sleep(0.05)  # Sleep time necessary to read measures of the gripable device
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

        if abs(mean_acc_x) > acc_min:     # The user is bending the GripAble forward or backward
            move_forward_reverse(mean_acc_x)
            move_right_left(mean_acc_y)

        elif abs(mean_acc_x) < acc_min < abs(mean_acc_y):
            motor_forward_reverse.ChangeDutyCycle(ref_Speed)
            spin_in_place(mean_acc_y)

        else:                             # The user is not bending the GripAble device
            motor_forward_reverse.ChangeDutyCycle(ref_Speed)
            motor_right_left.ChangeDutyCycle(ref_Direction)

    except KeyboardInterrupt:
        break

# Stop connections with the GripAble device and clean the gpio configuration
print("Joystick movement program stopped")
gpio.cleanup()
flex.stop()
flex.join()
