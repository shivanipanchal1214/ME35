from machine import RTC, PWM, Pin
from time import sleep

rtc = RTC()
rtc.datetime((2026,9,16,0,11,59,55,0))
print(rtc.datetime())

servo1 = PWM(Pin(4), freq=50, duty_u16=0)
servo2 = PWM(Pin(18), freq=50, duty_u16=0)

angle_hour = 0
angle_min = 0

while True:
    current = rtc.datetime()
    hour = current[4]
    minute = current[5]
    second = current[6]
    if hour == 0:
        hour == 12
        print(hour, minute, second, "AM")
    elif hour == 12:
        print(hour, minute, second, "PM")
    elif hour > 12:
        hour = hour-12
        print(hour, minute, second,"PM")
    else:
        print(hour,minute,second, "AM")


    angle_hour = (hour%12)*15
#     angle = max(0, min(180, angle))
    pulse_hour = 2500 - (((2*angle_hour)/180)*1000)
    servo1.duty_ns(int(pulse_hour*1000))
    if hour == 12:
        angle_hour = 0
    
    if minute <= 30:
        angle_min = minute*6
    else:
        angle_min = 180 - (minute - 30)*6
    pulse_min = 2500 - (((2*angle_min)/180)*1000)
    servo2.duty_ns(int(pulse_min*1000))
    sleep(1)
    
    
    

