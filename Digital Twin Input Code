from machine import Pin
from machine import PWM
import math
import time

import urequests
import network
import json
import ubinascii

# establish wifi connection
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
    print("Connected! IP address:", wlan.ifconfig()[0])
    return wlan

# post an angle to a mate in the assembly
def set_revolute(did, wid, eid, mate_name, feature_id, angle_rad):
    url = "https://cad.onshape.com/api/v17/assemblies/d/{}/w/{}/e/{}/matevalues".format(did, wid, eid)
    body = {"mateValues": [{
        "jsonType": "Revolute",
        "mateName": mate_name,
        "featureId": feature_id,
        "ownerOccurrencePath": [],
        "rotationZ": angle_rad,
    }]}
    r = urequests.post(url, headers=headers, data=json.dumps(body))
    r.close()

# function for converting angles in degrees to position servo
def deg2serv(theta):
    theta = theta/180 * 2 + 0.5
    return int(theta*65535/20)

# define encoder class
class Count(object):
    def __init__(self,A,B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0

        self.A.irq(self.cb,self.A.IRQ_FALLING|self.A.IRQ_RISING) #interrupt on line A
        self.B.irq(self.cb,self.B.IRQ_FALLING|self.B.IRQ_RISING) #interrupt on line B


    def cb(self,msg):
        other,inc = (self.B,1) if msg == self.A else (self.A,-1) #define other line and increment
        self.counter += -inc if msg.value()!=other.value() else  inc

    def value(self):
        return self.counter

SSID = "tufts_eecs" #use tufts_eecs
PASSWORD = "foundedin1883" #foundedin1883

# Assemble the URL for the API call
api_url = "https://cad.onshape.com/api/v17/documents/774d39bdec33d0c04457313f"

# keys
access_key = "on_0zungLfKlzhyd9Z4ucUKK"
secret_key = "7pLpqvOv3WfEQSV7LSgAYtQn7DD4kDccRdeCJGybQsyVYLO2"

# Define the header for the request
token = ubinascii.b2a_base64("{}:{}".format(access_key, secret_key)).decode().strip()
headers = {'Accept': 'application/json;charset=UTF-8;qs=0.09',
           'Content-Type': 'application/json',
           'Authorization': 'Basic ' + token}

connect_wifi()

# Putting everything together to make the API request
response = urequests.get(api_url, headers=headers)
doc = response.json()
response.close()

print(doc["name"])
did = '774d39bdec33d0c04457313f'
wid = 'e435f4bc1e9dbd83b50ffd84'

eid = "a52a0be965b84de14e97ad69"
url = "https://cad.onshape.com/api/v17/assemblies/d/{}/w/{}/e/{}/matevalues".format(did, wid, eid)
response = urequests.get(url, headers=headers)
data = response.json()
response.close()

# mate IDs
ELBOW_ID = "MXls5M0KR225T75yY"
SHOULDER_ID = "MiWNkNLjsmBOJ+uD3"

# turn on LED
led = Pin(12,Pin.OUT)
led.on()

# linkage lengths
l1 = 66 # mm
l2 = 70 # mm

# init shoulder servo
shoulder = PWM(Pin(4), freq=50, duty_u16=0)
shoulder.duty_u16(deg2serv(180)) # zero
shoulderServPos = 180

# init elbow servo
elbow = PWM(Pin(18), freq=50, duty_u16=0)
elbow.duty_u16(deg2serv(0)) # zero
elbowServPos = 0

shCount = Count(22,21) # Shoulder DC Motor Position
oldShPos = shCount.value()

elbCount = Count(26,25) # Elbow DC Motor Position
oldElbPos = elbCount.value()

set_revolute(did, wid, eid, "Shoulder", SHOULDER_ID, -shoulderServPos/180*math.pi)
set_revolute(did, wid, eid, "Elbow", ELBOW_ID, -elbowServPos/180*math.pi)

# code for moving the arm with the wheel
while True:
    # record current positions
    shPos = shCount.value()
    elbPos = elbCount.value()

    dShWheel = (shPos - oldShPos)/500*90 # how turned was the shoulder wheel
    dElbWheel = (elbPos - oldElbPos)/500*90 # how turned was the elbow wheel

    if dShWheel != 0 or dElbWheel != 0:
        shoulderServPos += dShWheel
        elbowServPos += dElbWheel

        # don't try to set the servos to positions they cant go
        if shoulderServPos >= 180:
            shoulderServPos = 180
        elif shoulderServPos <= 0:
            shoulderServPos = 0

        if elbowServPos >= 180:
            elbowServPos = 180
        elif elbowServPos <= 0:
            elbowServPos = 0

        # move servos
        shoulder.duty_u16(deg2serv(shoulderServPos))
        elbow.duty_u16(deg2serv(elbowServPos))
        oldShPos = shPos # record old positions
        oldElbPos = elbPos

        time.sleep(0.9)
        if shCount.value() == shPos and elbCount.value() == elbPos:
            # send to onshape
            set_revolute(did, wid, eid, "Shoulder", SHOULDER_ID, -shoulderServPos/180*math.pi)
            set_revolute(did, wid, eid, "Elbow", ELBOW_ID, -elbowServPos/180*math.pi)

    # print for debugging
    print("Shoulder:", shoulderServPos, dShWheel, "Elbow:", elbowServPos, dElbWheel)
    time.sleep(0.1) # make smaller to smooth motion
