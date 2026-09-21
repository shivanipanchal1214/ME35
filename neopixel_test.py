import neopixel #importing the library
from machine import Pin # another way of importing a library
import time
lights = neopixel.NeoPixel(Pin(15),2) # 15 is the Pin for neopixel and 2 is the number of lights

while True:
    lights[0] = (20,0,20) # set the color of 0th light to purple
    lights[1] = (0,128,0) #green
    lights.write()
    time.sleep(0.5)
    
    lights[0] = (0,128,0) # set the color of 0th light to purple
    lights[1] = (20,0,20) #green
    lights.write()
    time.sleep(0.5)