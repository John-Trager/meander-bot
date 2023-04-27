#!/usr/bin/env python3

import rospy
import board
import busio
from motor_controller.msg import MotorCommand
from adafruit_motorkit import MotorKit

# create motorkit
i2c = busio.I2C(board.SCL, board.SDA)
kit = MotorKit(i2c=i2c)
kit.motor1.throttle = 0
kit.motor2.throttle = 0

# Define the callback function to handle incoming motor commands
def motor_command_callback(data):
    # Update motor speeds based on incoming command
    kit.motor1.throttle = data.motor1_speed
    kit.motor2.throttle = data.motor2_speed


if __name__ == '__main__':
    rospy.init_node('motor_controller')

    # Set up the subscriber to listen for motor commands
    rospy.Subscriber('motor_commands', MotorCommand, motor_command_callback)

    # Spin the node to listen for messages and update motor speeds accordingly
    rospy.spin()