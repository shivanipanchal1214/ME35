import network, time, math, ubinascii
from machine import Pin, PWM
import secrets

try:
    import requests
except ImportError:
    import urequests as requests

DID = "774d39bdec33d0c04457313f"
WID = "e435f4bc1e9dbd83b50ffd84"
EID = "a52a0be965b84de14e97ad69"
URL = "https://cad.onshape.com/api/v10/assemblies/d/{}/w/{}/e/{}/matevalues".format(DID, WID, EID)
DELAY = 1   # seconds between reads

SHOULDER_PIN = 4
ELBOW_PIN = 5
ELBOW_OFFSET = 0
# shoulder in onshape moves 180-360 so need to offset
SHOULDER_OFFSET = 180
SHOULDER_SIGN = -1      # set to -1 if the shoulder turns the wrong way
ELBOW_SIGN = 1          # set to -1 if the elbow turns the wrong way
ELBOW_RATIO = 0.5       # 2:1 gearing: 0-360 joint -> 0-180 servo

token = ubinascii.b2a_base64("{}:{}".format(secrets.ACCESSKEY, secrets.SECRETKEY)).decode().strip()
HEADERS = {'Accept': 'application/json;charset=UTF-8;qs=0.09',
           'Content-Type': 'application/json',
           'Authorization': 'Basic ' + token}

# --- servos
shoulder = PWM(Pin(SHOULDER_PIN), freq=50, duty_u16=0)
elbow = PWM(Pin(ELBOW_PIN), freq=50, duty_u16=0)

def deg2serv(theta):
    theta = max(0, min(180, theta))           # never command past the servo's range
    return int((theta / 180 * 2 + 0.5) * 65535 / 20)   # 0.5-2.5 ms pulse

def shoulder_angle(deg):
    # map onshape shoulder angle to physical joint
    deg = deg % 360
    if deg < 90:
        deg += 360
    return max(0, min(180, deg - 180))

def elbow_angle(deg):
    return deg % 360

def move(sh_deg, el_deg):
    sh_servo = SHOULDER_OFFSET + SHOULDER_SIGN * sh_deg
    el_servo = ELBOW_OFFSET + ELBOW_SIGN * el_deg * ELBOW_RATIO
    shoulder.duty_u16(deg2serv(sh_servo))
    elbow.duty_u16(deg2serv(el_servo))
    return sh_servo, el_servo

def get_mates():
    try:
        try:
            r = requests.get(URL, headers=HEADERS, timeout=10)   # don't hang forever
        except TypeError:
            r = requests.get(URL, headers=HEADERS)               # older urequests: no timeout arg
    except OSError as e:
        print("Request failed:", e)
        return None
    try:
        if r.status_code != 200:
            print("Status", r.status_code, r.text[:200])
            return None
        return r.json()["mateValues"]
    finally:
        r.close()

# connecting to wifi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print("Connecting to WiFi...")
    wlan.connect(secrets.SSID, secrets.PWD)
    for _ in range(40):
        if wlan.isconnected():
            break
        time.sleep(0.5)
if not wlan.isconnected():
    raise RuntimeError("WiFi connection failed")
print("WiFi connected:", wlan.ifconfig()[0])

# reading loop
n = 0
last = None
try:
    while True:
        n += 1
        mates = get_mates()
        if mates:
            by_name = {m.get("mateName"): m for m in mates}
            if "Shoulder" in by_name and "Elbow" in by_name:
                raw_sh = math.degrees(by_name["Shoulder"].get("rotationZ", 0.0))
                raw_el = math.degrees(by_name["Elbow"].get("rotationZ", 0.0))
                sh, el = shoulder_angle(raw_sh), elbow_angle(raw_el)
                if last is None or abs(sh - last[0]) > 0.5 or abs(el - last[1]) > 0.5:
                    s1, s2 = move(sh, el)
                    last = (sh, el)
                    print("[{}] MOVE  Shoulder {:7.2f}  Elbow {:7.2f}  ->  servo4 {:6.1f}  servo5 {:6.1f}".format(
                        n, sh, el, s1, s2))
                else:
                    print("[{}] no change  (Shoulder {:.1f}, Elbow {:.1f})".format(n, sh, el))
            else:
                print("Mates found:", list(by_name.keys()), "- expected 'Shoulder' and 'Elbow'")
        time.sleep(DELAY)
except KeyboardInterrupt:
    shoulder.deinit()
    elbow.deinit()
    print("Stopped")
