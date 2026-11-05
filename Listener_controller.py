# controller.py -- MicroPython on the controller ESP32 (BLE central / "Listener")
# Needs BLE_CEEO.py on this board's flash.
#
# Sends   : "J,x,y,boost\n" to the car about 20 times per second
# Receives: "F\n" from the car when it crosses the red tape -> celebrate()

from BLE_CEEO import Listen
from machine import Pin, ADC, PWM
import time

# ======================= CONFIG =======================
NAME = "ESP_Controller"                 # must match the car's NAME exactly (<= 8 chars, unique to your team)

JOY_X_PIN = 34                 # ADC input-only pins are fine for the joystick
JOY_Y_PIN = 35
BOOST_PIN = 25                 # buttons: use pins WITH internal pull-ups (not 34-39)
HORN_PIN = 26
LED_PIN = 27                   # boost LEDs (swap for NeoPixel code if you use those)
SPEAKER_PIN = 33               # speaker / buzzer for the airhorn and the celebration

DEADZONE = 8                   # joystick values smaller than this count as 0
SEND_MS = 50                   # about 20 updates per second
# ======================================================

joy_x = ADC(Pin(JOY_X_PIN)); joy_x.atten(ADC.ATTN_11DB)
joy_y = ADC(Pin(JOY_Y_PIN)); joy_y.atten(ADC.ATTN_11DB)
boost_btn = Pin(BOOST_PIN, Pin.IN, Pin.PULL_UP)
horn_btn = Pin(HORN_PIN, Pin.IN, Pin.PULL_UP)
led = Pin(LED_PIN, Pin.OUT)

finished = False


def callback(data):
    # Runs in IRQ context: just set a flag, handle it in the main loop
    global finished
    if b"F" in data:
        finished = True


def scale(raw):
    """0..4095 -> -100..100 with a deadzone around the center."""
    v = (raw - 2048) * 100 // 2048
    return 0 if abs(v) < DEADZONE else max(-100, min(100, v))


def tone(freq, ms):
    spk = PWM(Pin(SPEAKER_PIN), freq=freq, duty_u16=32768)
    time.sleep_ms(ms)
    spk.deinit()


def airhorn():
    # Placeholder: replace with your existing airhorn code (user-input sound)
    tone(440, 120)


def celebrate():
    # Placeholder victory: flash the LEDs and play a rising tune
    for freq in (523, 659, 784, 1046):
        led.value(1)
        tone(freq, 150)
        led.value(0)
        time.sleep_ms(40)
    for _ in range(4):
        led.value(1)
        time.sleep_ms(100)
        led.value(0)
        time.sleep_ms(100)


p = None
try:
    p = Listen(NAME, verbose=False)
    p.callback = callback

    while True:
        if not p.is_connected:
            led.value(0)
            print("Looking for", NAME, "...")
            p.connect_up(timeout=5000)        # scan up to 5 s, then try again
            continue

        boost = 1 if boost_btn.value() == 0 else 0
        led.value(boost)                      # boost LEDs on while the button is held

        x = scale(joy_x.read())
        y = -scale(joy_y.read())              # flip so pushing the stick forward is positive
        p.send("J,%d,%d,%d\n" % (x, y, boost))

        if horn_btn.value() == 0:
            airhorn()                         # note: this blocks while it plays, so keep it short

        if finished:
            finished = False
            celebrate()

        time.sleep_ms(SEND_MS)
finally:
    if p:
        p.disconnect()