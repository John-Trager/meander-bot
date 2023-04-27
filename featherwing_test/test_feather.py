"""Simple test for using adafruit_motorkit with a DC motor"""
import time
import board
from adafruit_motorkit import MotorKit

print('Starting Feather Motor test on motor 1')
print('Loading MotorKit')

kit = MotorKit(i2c=board.I2C())

print('Test starting...')

kit.motor1.throttle = 1.0
time.sleep(2)
kit.motor1.throttle = 0
time.sleep(1)
kit.motor1.throttle = -1.0
time.sleep(2)
kit.motor1.throttle = 0

print('Test ending...')
