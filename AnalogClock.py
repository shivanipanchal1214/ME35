from machine import RTC, PWM, Pin
from time import sleep

rtc = RTC()
rtc.datetime((2026,9,16,0,12,59,58,0))
print(rtc.datetime())

servo = PWM(Pin(4), freq=50, duty_u16=0)

while True:
    current = rtc.datetime()
    hour = current[4]
    minute = current[5]
    second = current[6]
    if hour > 12:
        hour = hour - 12
    if hour == 0:
        hour = 12
     print(hour,minute,second)

    angle = hour*15
    angle = max(0, min(180, angle))
    pulse = 2500 - (((2*angle)/180)*1000)
    servo.duty_ns(int(pulse*1000))
    sleep(1)
    

