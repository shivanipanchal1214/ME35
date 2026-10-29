print("hello")
print("hi")
from machine import Pin
import time

led_mild = Pin(13, Pin.OUT)

while True:
    led_mild.on()
    print("LED should be ON")
    time.sleep(1)
    led_mild.off()
    print("LED should be OFF")
    time.sleep(1)