from machine import PWM, Pin, I2C
import time
from lis3dh_min import LIS3DH

servo1 = PWM(Pin(18), freq=50)
button = Pin(4, Pin.IN, Pin.PULL_UP)

#angle movement
initial_angle = 90
left_angle = 45
right_angle = 110
step = 3

def set_angle(angle):
    angle = max(0, min(180, angle))
    pulse = 500 + (angle / 180) * 2000
    servo1.duty_ns(int(pulse * 1000))

angle = initial_angle
direction = 1
moving = False
set_angle(angle)

#accelerometer detection
i2c = I2C(0, scl=Pin(33), sda=Pin(25))
acc = LIS3DH(i2c, 0x19)        

# while True:
#     x, y, z = acc.read_g()
#     print("x=%.2f y=%.2f z=%.2f" % (3*x, 3*y, 3*z))
#     time.sleep(0.7)


AXIS = 0            # 0=x
THRESH = 0.025       
NEEDED = 5         
 
led = Pin(26, Pin.OUT)                         
buzzer = PWM(Pin(23), freq=1000, duty_u16=0)

def averaged(n=3):
    total = 0
    for _ in range(n):
        total += acc.read_g()[AXIS]
        time.sleep(0.01)
    return total / n

def thwip():
    for f, ms in [(2500, 30), (1200, 50), (600, 60)]:  
        buzzer.freq(f)
        buzzer.duty_u16(30000)
        time.sleep_ms(ms)
    buzzer.duty_u16(0)
    
# Calibrate with the tongue at rest
print("calibrating, keep tongue still...")
baseline = averaged(30)
print("baseline:", baseline)


count = 0
buzz_on = False

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
        print("MOVING")
    
    delta = abs(averaged() - baseline)
    print("delta=%.3f" % delta)
    if delta > THRESH:
        count += 1
    else:
        count = 0

    if count >= NEEDED:                 # weight detected
        led.value(1)
        thwip()
        buzz_on = True
        print("led on")
    else:
        led.value(0)
        buzz_on = False

    time.sleep(0.1)
    

#Part 2

