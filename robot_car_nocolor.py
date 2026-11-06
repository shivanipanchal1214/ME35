# car_ble_test.py -- MicroPython on ESP32 + Cytron ROBO-ESP32 (BLE peripheral / "Yeller")
# Minimal test: connect to the controller over BLE and drive the motors with the joystick.
# No color sensors, no KNN. Needs BLE_CEEO.py on the car's flash.
#
# Controller sends: "J,x,y,boost\n"   x,y in -100..100, boost 0/1

from machine import Pin, PWM
import time
from BLE_CEEO import Yell

# ======================= CONFIG =======================
NAME = "ESP_SY"                  # must match the controller's NAME (<= 8 chars)
FAILSAFE_MS = 500                # no message for this long -> stop the motors
NORMAL_GAIN = 0.6                # max motor speed (0..1) without boost
BOOST_GAIN = 1.0                 # max motor speed (0..1) with boost

# Motor pins: M1 (Right) = D12/D13, M2 (Left) = D14/D27
RIGHT_A, RIGHT_B = 12, 13
LEFT_A,  LEFT_B  = 14, 27
RIGHT_INVERT = True
LEFT_INVERT  = False
# ======================================================


class Motor:
    def __init__(self, pa, pb, invert):
        self.a = PWM(Pin(pa), freq=20000, duty_u16=0)
        self.b = PWM(Pin(pb), freq=20000, duty_u16=0)
        self.invert = invert

    def set(self, speed):
        speed = max(-1.0, min(1.0, speed))
        if self.invert:
            speed = -speed
        duty = int(abs(speed) * 65535)
        if speed >= 0:
            self.a.duty_u16(duty)
            self.b.duty_u16(0)
        else:
            self.a.duty_u16(0)
            self.b.duty_u16(duty)


right = Motor(RIGHT_A, RIGHT_B, RIGHT_INVERT)
left = Motor(LEFT_A, LEFT_B, LEFT_INVERT)


def drive(l, r):
    left.set(l)
    right.set(r)


def stop():
    left.set(0)
    right.set(0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


buffer = b""
latest = None                    # most recent complete message line (bytes)


def callback(data):
    # Runs in IRQ context: only buffer and split. No motor work here.
    global buffer, latest
    buffer += data
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        latest = line


p = None
try:
    stop()
    p = Yell(NAME, interval_us=30000, verbose=False)
    p.callback = callback

    while True:
        stop()
        print("Waiting for the controller...")
        if not p.connect_up():
            continue
        print("Connected")

        last_msg = time.ticks_ms()
        while p.is_connected:
            if latest is not None:
                line, latest = latest, None
                try:
                    parts = line.decode().split(",")
                    if parts[0] == "J" and len(parts) == 4:
                        x, y, boost = int(parts[1]), int(parts[2]), int(parts[3])
                        gain = BOOST_GAIN if boost else NORMAL_GAIN
                        l = clamp((y + x) / 100.0, -1.0, 1.0) * gain
                        r = clamp((y - x) / 100.0, -1.0, 1.0) * gain
                        print("x=%d y=%d boost=%d -> L=%.2f R=%.2f" % (x, y, boost, l, r))
                        drive(l, r)
                        last_msg = time.ticks_ms()
                except (ValueError, UnicodeError):
                    pass                          # ignore a garbled message

            # failsafe: no message recently -> stop
            if time.ticks_diff(time.ticks_ms(), last_msg) > FAILSAFE_MS:
                stop()

            time.sleep_ms(10)

        print("Lost connection")
finally:
    stop()
    if p:
        p.disconnect()