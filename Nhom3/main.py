#!/usr/bin/env pybricks-micropython
#ptd
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
# More
# pip install python-ev3dev2
import ev3dev2

motora = MediumMotor(OUTPUT_A)
motorb = LargeMotor(OUTPUT_B)
sensor1 = ColorSensor(OUTPUT_1)
sensor2 = ColorSensor(OUTPUT_2)
ptd = EV3Brick()
error = proportional = derivative = 0

def ackermanPID(power, port1, port2, lower,kp,ki,kd):
    error = port1 - port2
    proportional = error * kp 
    integral += error
    integral *= ki
    derivative = last_error - error
    derivative *= kd
    pid = proportional + integral + derivative
    powerA = 2 * pid * lower
    motora.run_direct(duty_cycle_sp=powerA)
    powerB = power * -1
    motorb.run_direct(duty_cycle_sp=powerB)
    last_error = error

while (1):
    ackermanPID(70,sensor1.get_reflected_light(),sensor2.get_reflected_light(),-0.15,0.45,0.00007,80)

