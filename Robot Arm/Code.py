from machine import Pin
from machine import PWM
import math
import time

led = Pin(12,Pin.OUT)
time.sleep(2)
led.on()
# list of points for the tip of the arm to go to
points = [(64.453, 117.677), (70.277, 103.658), (76.100, 89.640), (73.594, 80.183), (64.453, 75.853), (55.754, 79.674), (53, 89.640), (57, 103.658), (64.453, 117.677)] # points for rain drop
# points = [(93, 32), (93, 61), (118, 61), (118, 32)] # points for square
angles = []

# linkage lengths
l1 = 66 # mm
l2 = 70 # mm

for p in points:
    (x, y) = p
    # throw error if point is out of range
    if x**2+y**2 < l1**2+l2**2 or math.sqrt(x**2+y**2) > l1+l2:
        raise ValueError("Point is out of arm reach")
    
    # inverse kinematics to find theta values
    costheta2 = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)
    sintheta2 = (math.sqrt(1-costheta2**2))
    theta2 = math.atan2(sintheta2,costheta2)
    
    theta1 = math.atan2(y, x)-math.atan2(l2*math.sin(theta2), l1+l2*math.cos(theta2))
    
    # if theta1 came out hugely negative, make it equivilant but positive
    if theta1 < -math.pi:
        theta1 += 2*math.pi
    
    # if theta1 is out of range, flip theta2's sign and resolve
    if theta1 < 0 or theta1 > 180:
        theta2 *= -1
        theta1 = math.atan2(y, x)-math.atan2(l2*math.sin(theta2), l1+l2*math.cos(theta2))
    
    # add to angles list
    angles.append((theta1/math.pi*180, theta2/math.pi*180))

# function for converting angles in degrees to position servo
def deg2serv(theta):
    theta = theta/180 * 2 + 0.5
    return int(theta*65535/20)

# init shoulder servo
shoulder = PWM(Pin(4), freq=50, duty_u16=0)
# shoulder.duty_u16(deg2serv(0)) # zero
shoulderServPos = 0

# init elbow servo
elbow = PWM(Pin(18), freq=50, duty_u16=0)
# elbow.duty_u16(deg2serv(0+90)) # zero
elbowServPos = 0

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
    
    
# shCount = Count(22,21) # Shoulder DC Motor Position
# oldShPos = shCount.value()
# 
# elbCount = Count(26,25) # Elbow DC Motor Position
# oldElbPos = elbCount.value()

#moves both servos to a target in small PWM steps instead of one big jump
def move_to(th1, th2, steps=50, delay=0.02):
    global shoulderServPos, elbowServPos
    s0 = shoulderServPos
    e0 = elbowServPos
    for i in range(1, steps+1):
        f = i/steps
        shoulder.duty_u16(deg2serv(s0 + (th1 - s0)*f))
        elbow.duty_u16(deg2serv(e0 + (th2 - e0)*f + 90))
        time.sleep(delay)
    shoulderServPos = th1
    elbowServPos = th2





for thetas in angles:
    # move servos and record position for wheel control
    move_to(thetas[0], thetas[1])
    
    # forward kinematics to make sure its where we think it is
    theta1 = thetas[0]*math.pi/180 # conv to rads
    theta2 = thetas[1]*math.pi/180
    
    x = l1*math.cos(theta1)+l2*math.cos(theta1+theta2)
    y = l1*math.sin(theta1)+l2*math.sin(theta1+theta2)
    print("Thetas:", thetas,"Position:",(x,y))
    

# code for moving the arm with the wheel
while False:
    # record current positions
    shPos = shCount.value()
    elbPos = elbCount.value()
    
    dShWheel = (shPos - oldShPos)/500*90 # how turned was the shoulder wheel
    dElbWheel = (elbPos - oldElbPos)/500*90 # how turned was the elbow wheel
    
    shoulderServPos += dShWheel
    elbowServPos += dElbWheel
    
    # don't try to set the servos to positions they cant go
    if shoulderServPos >= 180:
        shoulderServPos = 180
    elif shoulderServPos <= 0:
        shoulderServPos = 0
        
    if elbowServPos >= 90:
        elbowServPos = 90
    elif elbowServPos <= -90:
        elbowServPos = -90
    
    # move servos
    shoulder.duty_u16(deg2serv(shoulderServPos))
    elbow.duty_u16(deg2serv(elbowServPos+90))
    oldShPos = shPos # record old positions
    oldElbPos = elbPos
    
    # print for debugging
    print("Shoulder:", shPos, shoulderServPos, dShWheel, "Elbow:", elbPos, elbowServPos, dElbWheel) 
    time.sleep(0.1) # make smaller to smooth motion
    
    



