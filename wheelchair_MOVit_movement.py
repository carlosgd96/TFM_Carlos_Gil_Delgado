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
acc_min = 0.3  # Minimum acceleration which is detected by the GripAble device
goForward = True  # Variable to change between forward and reverse modes

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


# Function to limit values between min_acc and 1
def value_limits(value):
    if value > 1.0:
        value = 1.0
    if value < acc_min:
        value = 0
    return value * 100


def move_forward_reverse(velocity):
    # The ref_Direction is 66, the maximum value is 80 and the minimum 52.5, the 0.14 is the ratio to multiply
    # being 14 the maximum adding or reduction due to the range of motion is [30, 100]
    velocity = velocity * 0.14
    if goForward:
        motor_forward_reverse.ChangeDutyCycle(ref_Speed + velocity)
    else:
        motor_forward_reverse.ChangeDutyCycle(ref_Speed - velocity)


def move_right_left(rot):
    # The ref_Direction is 66, the maximum value is 80 and the minimum 52.5, the 0.14 is the ratio to multiply
    # being 14 the maximum adding or reduction due to the range of motion is [30, 100]
    rot = rot * 0.14
    if acc_right > acc_left:
        motor_right_left.ChangeDutyCycle(ref_Direction + rot)
    else:
        motor_right_left.ChangeDutyCycle(ref_Direction - rot)


def spin_in_place(rotation_right, rotation_left):
    # The ref_Direction is 66, the maximum value is 80 and the minimum 52.5, the 0.14 is the ratio to multiply
    # being 14 the maximum adding or reduction due to the range of motion is [30, 100]
    rotation_right = rotation_right * 0.14
    rotation_left = rotation_left * 0.14
    if mode == 2:
        motor_right_left.ChangeDutyCycle(ref_Direction + rotation_right)
    else:
        motor_right_left.ChangeDutyCycle(ref_Direction - rotation_left)


# Main loop
while True:
    try:
        time.sleep(0.05)  # Sleep time necessary to read measures of the gripable device
        devRdata: SensorData = flex_right.getSensorData()
        devLdata: SensorData = flex_left.getSensorData()

        # Saving acceleration and force values of both GripAbles
        acc_right_abs = abs(float(devRdata.accelerometer.x))
        acc_left_abs = abs(float(devLdata.accelerometer.x))
        force_right = devRdata.grip.force
        force_left = devLdata.grip.force

        # Limitation of the values between acc_min and 1
        acc_right_abs = value_limits(acc_right_abs)
        del prev_acc_right[0]
        prev_acc_right.append(acc_right_abs)
        acc_left_abs = value_limits(acc_left_abs)
        del prev_acc_left[0]
        prev_acc_left.append(acc_left_abs)

        # Computation of the mean values and limit both accelerations
        mean_acc_right = 2 * statistics.mean(prev_acc_right)
        mean_acc_right = value_limits(mean_acc_right)
        print("Mean acc right: %f \n" % mean_acc_right, end="")
        mean_acc_left = 2 * statistics.mean(prev_acc_left)
        mean_acc_left = value_limits(mean_acc_left)
        print("Mean acc left: %f \n" % mean_acc_left, end="")

        # Mode 1: forward and reverse, Mode 2: spin in place right direction, Mode 3: spin in place left direction
        mode = 1

        # If force is applied to both GripAble devices change between forward and reverse
        if (force_right > 1) and (force_left > 1):
            if goForward:
                goForward = False
                print("Reverse mode \n")
            else:
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

        rotation = abs(mean_acc_right - mean_acc_left)
        if mode == 1:  # Forward and reverse mode
            if (mean_acc_right > acc_min) or (mean_acc_left > acc_min):
                move_forward_reverse(statistics.mean(mean_acc_right, mean_acc_left))
                if rotation < 40:
                    motor_right_left.ChangeDutyCycle(ref_Direction)  # Reference value to do not apply voltage
                else:
                    move_right_left(rotation)

            else:
                motor_forward_reverse.ChangeDutyCycle(ref_Speed)     # Reference value to do not apply voltage
                motor_right_left.ChangeDutyCycle(ref_Direction)      # Reference value to do not apply voltage

        else:  # Spin in place to the right/left mode
            motor_forward_reverse.ChangeDutyCycle(ref_Speed)         # Reference value to do not apply voltage
            spin_in_place(mean_acc_right, mean_acc_left)

    except KeyboardInterrupt:
        break

# Stop connections with the GripAble devices
gpio.cleanup()
flex_right.stop()
flex_right.join()
flex_left.stop()
flex_left.join()
