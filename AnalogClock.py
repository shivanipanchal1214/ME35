from machine import RTC, PWM, Pin
import time
import urequests, secretsclock
import gc

#initial servo and LED pins
servo1 = PWM(Pin(4), freq=50, duty_u16=0)
servo2 = PWM(Pin(18), freq=50, duty_u16=0)
led_warm = Pin(13,Pin.OUT)
led_mild = Pin(2,Pin.OUT)
led_warm.off()
led_mild.off()

#wifi connecting for API key
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
url =f"https://api.openweathermap.org/data/2.5/weather?q=Queens,US-NY,840&appid={secretsclock.weather_api}"


#Part1
rtc = RTC()
rtc.datetime((2026,9,16,0,17,59,55,0))
print(rtc.datetime())

angle_hour = 0
angle_min = 0

first_check = False
weather_interval = 10 * 1000   # 10 seconds to start
weather_interval_normal = 60 * 1000 #60 second intervals
last_weather_check = time.ticks_ms()

while True:
    current = rtc.datetime()
    hour = current[4]
    minute = current[5]
    second = current[6]
    if hour == 0:
        hour = 12
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
    
#    API key Part 2    
    if time.ticks_diff(time.ticks_ms(),last_weather_check) > weather_interval:
        #print(gc.mem_free())
        try:
            response = urequests.get(url)
            #print(response.status_code)
            data = response.json()
            #print(data)
            response.close()
            gc.collect()
            #print(gc.mem_free())
            temp_kelvin = data['main']['temp']
            temp = (temp_kelvin - 273.15) * 1.8 + 32
            #print("Temp:", temp)
            
            #LED
            temp_high = 80
            temp_med = 70
            temp_low = 60

            if temp_med <= temp <= temp_high:
                led_warm.on()
                print("Weather is warm:", temp, "degrees Fahrenheit")
            elif temp_low <= temp < temp_med:
                led_mild.on()
                print("Weather is mild:", temp, "degrees Fahrenheit")
                
        except Exception as e:
            print("Updated weather check failed:", e)
            #print(gc.mem_free())

        last_weather_check = time.ticks_ms()

        if not first_check:
            weather_interval = weather_interval_normal
            first_check = True
        
    time.sleep(1)



