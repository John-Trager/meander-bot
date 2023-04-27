# Meander Bot

TODO:
- work on fixing permissions with Jetson.GPIO
- see if ros msg works
- test out sending vel commands to motor_controller

The goal of this project is to create a modified jetbot that can patrol or meander around my house.

Robot Hardware:
- jetson nano
- adafruit featherwing motorcontroller
- DG10D-E-PH motor with encoder
- Slamtex RPLidar A1 lidar
- Luxonis OAK-D steoreo AI camera
- MPU6050 imu

# Setup

## Jetson Setup
note: that I don't remeber but you need to configure the I2C pins (maybe some other pin thing) on the jetson, there is some config thing
[here](https://learn.adafruit.com/circuitpython-libraries-on-linux-and-the-nvidia-jetson-nano/initial-setup) may be helpful

we will also need to set the correct permissions so our user can access GPIO. To do this we need to add our user to the GPIO group and add a rules file from from [here](https://github.com/NVIDIA/jetson-gpio/blob/6cab53dc80f8f5ecd6257d90dc7c0c51cb5348a7/lib/python/Jetson/GPIO/99-gpio.rules#L2) into `/etc/udev/rules.d/`. 
Also:
```
sudo groupadd -f -r gpio
sudo usermod -a -G gpio your_user_name
```
Then log the user out and back in and should be good to go.

## I2C
to check if a device is connected via I2C use:<br>
`i2cdetect -y -r 1` to see I2C bus 1 (pin 3 SDA and 5 SCL)<br>
`i2cdetect -y -r 0` to see I2C bus 0 (pin 27 SDA and 28 SCL)<br>

oddly enough the i2c bussses are mislabled when using busio:
- to used bus0: `i2c = busio.I2C(board.SCL_1, board.SDA_1)`
- to used bus1: `i2c = busio.I2C(board.SCL, board.SDA)`



## Setting Jetson power setting
`sudo nvpmodel -q` to see current power setting<br>
`sudo nvpmodel -m0` for maxn power setting (10W)<br>
`sudo nvpmodel -m1` for 5W power setting (recommended when not using barrel connector)<br>

## installing ROS melodic on Jetson
[here](https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/)

# Motor drivers info
TODO: take some pictures and add here

// feather motor driving wiring
- 5v-12v input power
- 3.3v logic power (across from input power)
- communication
- output on long ends of controller

# IMU info
uses addres `68` on i2c

# ROS

`catkin_make` ro build/compile code

Some info about ROS can be found in [MRover's wiki](https://github.com/umrover/mrover-ros/wiki/4.-Fundamentals-of-ROS)

## Running stuff
to check running nodes: `rosnode list`

make sure you start the main ros node before running other nodes:<br>
`roscore`

running motorcontroller node:<br>
`rosrun esw motor_controller.py`

