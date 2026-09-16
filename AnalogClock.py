from machine import RTC, PWM, Pin
from time import sleep

rtc = RTC()
rtc.datetime((2026,9,16,0,12,59,58,0))
print(rtc.datetime())

# while True:
#     current = rtc.datetime()
#     hour = current[4]
#     minute = current[5]
#     second = current[6]
#     if hour > 12:
#         hour = hour - 12
#     print(hour,minute,second)
# sleep(1)
    
servo = PWM(Pin(4), freq=50, duty_u16=0)

hour=1
angle = hour*15
for i in range(13):
    pulse = 2500 - (((2*angle)/180)*1000)
    servo.duty_ns(int(pulse*1000))
    angle = angle + 15
    sleep(1)