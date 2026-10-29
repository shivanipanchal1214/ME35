from machine import PWM, Pin, I2C
import time
from lis3dh import LIS2DH_I2C


#servo motion from button input
servo1 = PWM(Pin(18), freq=50)
button = Pin(34, Pin.IN, Pin.PULL_UP)

initial_angle = 90
left_angle = 45
right_angle = 135
step = 1

def set_angle(angle):
    angle = max(0, min(180, angle))
    pulse = 500 + (angle / 180) * 2000
    servo1.duty_ns(int(pulse * 1000))

angle = initial_angle
direction = 1
moving = False
set_angle(angle)

while True:
    if button.value() == 0:
        moving = not moving
        time.sleep(0.05)
        while button.value() == 0:
            time.sleep(0.01)
        
    if moving:
        angle += direction*step
        if angle >= right_angle:
            angle = right_angle
            direction = -1
        elif angle <= left_angle:
            angle = left_angle
            direction = 1
        set_angle(angle)

    time.sleep(0.01)


    
