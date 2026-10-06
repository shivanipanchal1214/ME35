from machine import Pin, I2C, PWM
import neopixel
import time
import math
import json
import veml6040

# Globals
DATA_FILE = "training.json"

DEBOUNCE_MS = 100
last_press = 0

STATES = ("red","green","blue","black","yellow","play")
CURR_STATE = "red"
shift_flag = False
execute_flag = False

WHITE_MAX = 40000
BINS = {"red":(180, 55), "green":(105, 55),"blue":(0, 55),"other":(75, 55)}
COLORS = {"red": (50, 0, 0), "green": (0, 50, 0), "blue": (0, 0, 50), "black": (5, 5, 5), "yellow": (30, 30, 0), "play": (30, 30, 30)}
REST = (90,90)

# Functions
def read_color():
    sensor.trigger_measurement()
    time.sleep(0.25)
    return sensor.read_rgbw()

def load_data():
    try:
        with open(DATA_FILE) as f:
            d = json.load(f)
        print("loaded %d samples" % len(d))
        return d
    except (OSError, ValueError):
        print("no saved data, starting fresh")
        return []


def save_data():
    try:
        with open(DATA_FILE, "w") as f:
            json.dump(data, f)
    except OSError as e:
        print("save failed:", e)

def sort_class(color):
    if color not in BINS:
        color = "other"
    c, t = BINS[color]
    print("sorting %s - circular %d, tilt %d" % (color, c, t))

    tilt.duty_u16(degree2servo(REST[1]))
    time.sleep(0.3)
    circular.duty_u16(degree2servo(c))
    time.sleep(0.8)
    tilt.duty_u16(degree2servo(t))
    time.sleep(0.8)
    tilt.duty_u16(degree2servo(REST[1]))
    time.sleep(0.5)
    circular.duty_u16(degree2servo(REST[0]))
    time.sleep(0.8)
    
def counts():
    out = {}
    for d in data:
        out[d[4]] = out.get(d[4], 0) + 1
    return out

def degree2servo(angle):
    # assume angle between 0 and 180
    angle = max(0, min(180, angle))
    ms_pulse = angle / 90 + 0.5
    duty = ms_pulse / 20 * 65535
    return int(duty)


def button_handler(pin):
    global shift_flag, execute_flag, last_press
    now = time.ticks_ms()
    if time.ticks_diff(now, last_press) > DEBOUNCE_MS:
        last_press = now
        if pin == button_shift:
            shift_flag = True
        if pin == button_execute:
            execute_flag = True


def k_nearest_neighbor(w,x,y,z, k = 3, max_dist = 0.5):
    distances = []
    for index, d in enumerate(data):
        dist = math.sqrt((w-d[0])**2+(x-d[1])**2+(y-d[2])**2+(z-d[3])**2)
        distances.append([dist,d[4]])
    
    distances.sort()
    print("nearest %.4f  %s" % (distances[0][0], distances[0][1]))
    if distances[0][0] > max_dist:
        return "not sure"
    distances = distances[:k] #get k distances
    classes = []
    for dist in distances:
        classes.append(dist[1])
    print("k classes", classes)
    most_number_of_closest_classes = max(set(classes), key = classes.count)
    print("max classes ", most_number_of_closest_classes)
    
    return most_number_of_closest_classes

# Initialize/Setup
button_shift = Pin(34, Pin.IN, Pin.PULL_UP)
button_execute = Pin(35, Pin.IN, Pin.PULL_UP)

i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)

button_shift.irq(trigger=Pin.IRQ_RISING, handler=button_handler)
button_execute.irq(trigger=Pin.IRQ_RISING, handler=button_handler)
circular = PWM(Pin(19),freq=50)
tilt = PWM(Pin(18),freq=50)
circular.duty_u16(degree2servo(REST[0]))
tilt.duty_u16(degree2servo(REST[1]))
pixels = neopixel.NeoPixel(Pin(15), 1)
pixels[0] = COLORS[CURR_STATE]
pixels.write()
time.sleep(1)

sensor = veml6040.VEML6040(i2c)
sensor.set_integration_time(veml6040.IT_160MS)
sensor.set_force_mode()
time.sleep(0.5)
print("state:", CURR_STATE)
       
data = load_data()

# While Loop
while True:
    red, green, blue, white = read_color()
    total = red + green + blue
    if total == 0:
        time.sleep(0.1)
        continue
    rn, gn, bn = red / total, green / total, blue / total
    wn = white / WHITE_MAX
    
    if shift_flag:
        shift_flag = False
        CURR_STATE = STATES[(STATES.index(CURR_STATE) + 1) % len(STATES)]
        pixels[0] = COLORS[CURR_STATE]
        pixels.write()
        print("state:", CURR_STATE)

    if execute_flag:
        execute_flag = False
        if CURR_STATE == "play":
            what_class = k_nearest_neighbor(rn, gn, bn, wn, 3)
            print(what_class)
            sort_class(what_class)
            execute_flag = False
            shift_flag = False
        else:
            data.append((rn, gn, bn, wn, CURR_STATE))
            save_data()
            print("%s   %s" % (CURR_STATE, counts()))
    time.sleep(0.1)
    
circular.deinit()
tilt.deinit()


