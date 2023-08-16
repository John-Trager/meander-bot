#!/usr/bin/env python3

import rospy
import board
import busio
# TODO: broken? can't find / import message file
#from esw.msg import MotorCommand
from geometry_msgs.msg import Twist

from adafruit_motorkit import MotorKit

# create motorkit
i2c = busio.I2C(board.SCL, board.SDA)
kit = MotorKit(i2c=i2c)
kit.motor1.throttle = 0
kit.motor2.throttle = 0

# Define the callback function to handle incoming motor commands
def motor_command_callback(msg):
    # Extract linear and angular velocities from cmd_vel message

    # TODO: still seems there could be a better way to do this such
    # as using a PID or something else for controlling the throttle
    # as ultimately we want to control the velocity not "throttle"

    # Map linear.x to throttle for both motors
    motor1_throttle = msg.linear.x
    motor2_throttle = msg.linear.x

    # Map angular.z to differential throttle for turning
    motor1_throttle -= msg.angular.z
    motor2_throttle += msg.angular.z

    # Clamp throttle values to range [-1, 1]
    motor1_throttle = max(-1, min(1, motor1_throttle))
    motor2_throttle = max(-1, min(1, motor2_throttle))

    # set actual motor throttle value
    kit.motor1.throttle = motor1_throttle
    kit.motor2.throttle = motor2_throttle

    print("Throttle (motor1, motor2):")
    print(motor1_throttle, motor2_throttle)

if __name__ == '__main__':
    rospy.init_node('motor_controller')

    # Set up the subscriber to listen for motor commands
    rospy.Subscriber('cmd_vel', Twist, motor_command_callback)

    # Spin the node to listen for messages and update motor speeds accordingly
    rospy.spin()

    # turn off the motors! (on exit of program)
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0