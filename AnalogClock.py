from machine import RTC, PWM, Pin
from time import sleep

rtc = RTC()
rtc.datetime((2026,9,16,0,3,59,55,0))
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
    
    angle_min = minute*3
#     angle = max(0, min(180, angle))
    pulse_min = 2500 - (((2*angle_min)/180)*1000)
    servo2.duty_ns(int(pulse_min*1000))
    if minute == 0:
        angle_min = 0
    sleep(1)
    
    
#     if minute <= 30:
#         angle_min = minute*6
#     else:
#         angle_min = 180 - (minute - 30)*6
#     pulse_min = 2500 - (((2*angle_min)/180)*1000)
#     servo2.duty_ns(int(pulse_min*1000))
#     sleep(1)


#API key Part 2
from Day3 import urequests, secretsclock

def wifi_connect():
    import network
    
    sta_if = network.WLAN(network.WLAN.IF_STA)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(secretsclock.SSID, secretsclock.PWD)
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ipconfig('addr4'))
    
wifi_connect()

url =f"https://api.openweathermap.org/data/2.5/forecast?q=Queens,US-NY,840&appid={secretsclock.weather_api}"

response = urequests.get(url)

print(response.status_code)
if response.status code == 200:
    data = response.json()
    print(data)
    
#### 
response.close()             # free the socket

temp = data['main']['temp']
#condition = data['weather'][0]['main']       # e.g. "Rain", "Clear", "Clouds"
temp_cel = temp - 273.15

print("Temp:", temp)
print("Condition:", condition)

led = LED(17) #check pin
temp_high = 80
temp_med = 60


#LED
from LED import gpio
if temp_med < temp < temp_high:
    led.on()
    print("Condition met: LED is on")



