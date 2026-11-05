# Yeller (peripheral)

from BLE_CEEO import Yell
import time
import json

NAME = "Maria"
buffer = b""


def callback(data):
    # Pieces arrive in up to 20-byte chunks; a newline ends each message
    global buffer
    buffer += data
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        print("Received:", json.loads(line.decode()))


p = None
try:
    p = Yell(NAME, interval_us=30000, verbose=False)
    p.callback = callback

    print("Waiting for a listener...")
    if p.connect_up():
        print("Connected")
        count = 0
        while p.is_connected:
            count += 1
            payload = json.dumps({"from": "yeller", "count": count}) + "\n"

            # Send in 20-byte pieces (the default packet size)
            for start in range(0, len(payload), 20):
                p.send(payload[start:start + 20])
                time.sleep_ms(20)

            time.sleep(1)
    print("Lost connection")
finally:
    if p:
        p.disconnect()
# car.py -- MicroPython on ESP32 + Cytron ROBO-ESP32 (BLE peripheral / "Yeller")
# Joystick drive over BLE + KNN red-tape finish-line detection that messages the controller.
# Needs on the car's flash: BLE_CEEO.py and knn_train.csv (made by MODE = "calibrate").
#
# Messages:
#   controller -> car : "J,x,y,boost\n"   x,y in -100..100, boost 0/1
#   car -> controller : "F\n"             sent when the red finish line is detected

from machine import Pin, PWM, SoftI2C
import neopixel
import time
import math
import gc

# Options: "calibrate" -> record KNN training samples (saves knn_train.csv), then evaluate
#          "evaluate"  -> re-test saved samples with current settings
#          "sensors"   -> live KNN readout for tuning
#          "motors"    -> test wheel direction (both should roll forward)
#          "run"       -> BLE joystick driving + finish-line detection
MODE = "run"

# ======================= CONFIG =======================
# BLE
NAME = "Maria"                   # must match the controller; <= 8 chars. Make it unique to your team!
FAILSAFE_MS = 500                # no joystick message for this long -> stop the motors
NORMAL_GAIN = 0.6                # max motor speed (0..1) without boost
BOOST_GAIN = 1.0                 # max motor speed (0..1) with boost

# Motor pins: M1 (Right) = D12/D13, M2 (Left) = D14/D27
RIGHT_A, RIGHT_B = 12, 13
LEFT_A,  LEFT_B  = 14, 27
RIGHT_INVERT = True              # Inverted: M1B drives CW forward
LEFT_INVERT  = False             # M2A drives CCW forward

# Dual Grove Port assignments (both sensors use fixed I2C address 0x10)
L_SDA, L_SCL = 21, 22            # Grove 2 (Left Sensor)
R_SDA, R_SCL = 26, 25            # Grove 3 (Right Sensor)

# Cytron Robo-ESP32 onboard buzzer & NeoPixels
BUZZER_PIN = 23
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

LOOP_MS = 40                     # main loop period (VEML6040 40 ms integration)

# KNN color detection (unchanged from your v9)
KNN_K = 5
KNN_MIN_VOTES = 4
MIN_VOTES = {"RED": 4, "GREEN": 5, "BLUE": 5}
KNN_MAX_DIST = 0.12
BRIGHT_W = 0.10
SAMPLES_PER_CLASS = 60
RUNTIME_STRIDE = 1               # set 2 or 3 if the loop is too slow
TRAIN_FILE = "knn_train.csv"
SCALE = 10000
LABELS = ("WHITE", "LINE", "RED", "GREEN", "BLUE")
COLORS = ("RED", "GREEN", "BLUE")

CONFIRM_RED = 2                  # consecutive RED readings before the finish triggers
COOLDOWN_MS = 2000               # ignore the tape for this long after finishing

# Black/white brightness calibration (W channel): (w_black, w_white, ...)
CAL = {
    'L': (250.7, 397.0, (0.9532228, 0.84450968, 1.3041067)),
    'R': (285.1, 393.9, (0.92870264, 0.8217886, 1.4156862))
}
# ======================================================


# ---------- Motors ----------
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


# ---------- NeoPixels ----------
np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)


def np_set(r, g, b):
    for i in range(NUM_PIXELS):
        np[i] = (r, g, b)
    np.write()


def np_clear():
    np_set(0, 0, 0)


np_clear()


# ---------- VEML6040 Driver ----------
class VEML6040:
    ADDR = 0x10

    def __init__(self, i2c, it=0):  # 40ms integration
        self.i2c = i2c
        self.i2c.writeto_mem(self.ADDR, 0x00, bytes([(it << 4), 0x00]))

    def _rd(self, reg):
        d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
        return d[0] | (d[1] << 8)

    def read(self):
        return self._rd(0x08), self._rd(0x09), self._rd(0x0A), self._rd(0x0B)


def make_sensors():
    i2c_l = SoftI2C(sda=Pin(L_SDA), scl=Pin(L_SCL), freq=100000)
    i2c_r = SoftI2C(sda=Pin(R_SDA), scl=Pin(R_SCL), freq=100000)
    return VEML6040(i2c_l), VEML6040(i2c_r)


# ---------- Helpers ----------
def refl(w, black, white):
    if white <= black:
        return 0.0
    return max(0.0, min(1.0, (w - black) / (white - black)))


def _chroma(rgb):
    t = rgb[0] + rgb[1] + rgb[2]
    if t <= 0:
        return None
    return (rgb[0] / t, rgb[1] / t, rgb[2] / t)


def wait_loop(t0):
    """Sleep out the remainder of LOOP_MS so KNN compute time doesn't slow the loop."""
    rest = LOOP_MS - time.ticks_diff(time.ticks_ms(), t0)
    if rest > 0:
        time.sleep_ms(rest)


# ---------- KNN ----------
TRAIN = {"L": [], "R": []}
LABEL_MAP = {lab: lab for lab in LABELS}


def load_training():
    gc.collect()
    n = {"L": 0, "R": 0}
    try:
        f = open(TRAIN_FILE)
    except OSError:
        raise RuntimeError("No %s found. Run MODE = \"calibrate\" first." % TRAIN_FILE)
    with f:
        for line in f:
            if line[:1] in n:
                n[line[:1]] += 1
    rows = {"L": [None] * (n["L"] // RUNTIME_STRIDE), "R": [None] * (n["R"] // RUNTIME_STRIDE)}
    cnt = {"L": 0, "R": 0}
    filled = {"L": 0, "R": 0}

    with open(TRAIN_FILE) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) != 6 or p[0] not in rows or p[5] not in LABEL_MAP:
                continue
            side = p[0]
            cnt[side] += 1
            if cnt[side] % RUNTIME_STRIDE:
                continue
            k = filled[side]
            if k >= len(rows[side]):
                continue
            rows[side][k] = (int(float(p[1]) * SCALE), int(float(p[2]) * SCALE),
                             int(float(p[3]) * SCALE), int(float(p[4]) * BRIGHT_W * SCALE),
                             LABEL_MAP[p[5]])
            filled[side] = k + 1
    for side in ("L", "R"):
        if filled[side] < len(rows[side]):
            rows[side] = rows[side][:filled[side]]
    TRAIN["L"], TRAIN["R"] = rows["L"], rows["R"]
    gc.collect()
    print("Loaded KNN data: L=%d  R=%d samples (free RAM %d bytes)" % (len(TRAIN["L"]), len(TRAIN["R"]), gc.mem_free()))


def _knn_core(c0, c1, c2, q, side, skip=-1):
    """Integer KNN. Returns (color or None, winning_label, votes, mean_distance in chroma units)."""
    train = TRAIN[side]
    if not train:
        return None, "---", 0, 9.0
    k = KNN_K
    best = []
    worst = 0x3FFFFFFF
    i = -1
    for a, b, cc, rf, lab in train:
        i += 1
        if i == skip:
            continue
        d = c0 - a
        d2 = d * d
        if d2 >= worst:
            continue
        d = c1 - b
        d2 += d * d
        if d2 >= worst:
            continue
        d = c2 - cc
        d2 += d * d
        if d2 >= worst:
            continue
        d = q - rf
        d2 += d * d
        if d2 >= worst:
            continue
        if len(best) < k:
            best.append((d2, lab))
            if len(best) == k:
                best.sort()
                worst = best[-1][0]
        else:
            best[-1] = (d2, lab)
            best.sort()
            worst = best[-1][0]
    best.sort()

    counts = {}
    for d2, lab in best:
        counts[lab] = counts.get(lab, 0) + 1
    top = max(counts.values())
    winner = None
    for d2, lab in best:
        if counts[lab] == top:
            winner = lab
            break
    ds = [math.sqrt(d2) for d2, lab in best if lab == winner]
    mean_d = sum(ds) / len(ds) / SCALE

    if winner in COLORS and top >= MIN_VOTES.get(winner, KNN_MIN_VOTES) and mean_d < KNN_MAX_DIST:
        return winner, winner, top, mean_d
    return None, winner, top, mean_d


def knn_classify(rgb, w_refl, side):
    c = _chroma(rgb)
    if c is None:
        return None, "---", 0, 9.0
    return _knn_core(int(c[0] * SCALE), int(c[1] * SCALE), int(c[2] * SCALE),
                     int(w_refl * BRIGHT_W * SCALE), side)


def benchmark_knn():
    qs = {}
    for side in ("L", "R"):
        rows = [r for r in TRAIN[side] if r[4] == "LINE"] or TRAIN[side]
        qs[side] = rows[len(rows) // 2]
    n = 10
    t0 = time.ticks_us()
    for _ in range(n):
        for side in ("L", "R"):
            a, b, cc, rf, _lab = qs[side]
            _knn_core(a, b, cc, rf, side)
    ms = time.ticks_diff(time.ticks_us(), t0) / n / 1000
    print("KNN cost: %.1f ms per loop (both sensors), loop target %d ms" % (ms, LOOP_MS))
    if ms > LOOP_MS * 0.5:
        print("  That's heavy. Set RUNTIME_STRIDE = 2 (or 3) to use every Nth sample at runtime.")


def calibrate_knn():
    sL, sR = make_sensors()
    prompts = {
        "WHITE": "robot on bare WHITE floor (slide it around a little)",
        "LINE": "robot centered on the BLACK LINE; wiggle it ~1 cm left/right along the tape",
        "RED": "robot centered on the RED patch; only a FEW MM of wiggle, keep both sensors on it",
        "GREEN": "robot centered on the GREEN patch; only a FEW MM of wiggle, keep both sensors on it",
        "BLUE": "robot centered on the BLUE patch; only a FEW MM of wiggle, keep both sensors on it",
    }
    secs = SAMPLES_PER_CLASS * 0.045
    print("\nKeep the real height and orientation. Each class samples for ~%.1f s:" % secs)
    print("LINE / WHITE: move it around the way the robot will see it. Colors: only a few mm, so the")
    print("samples stay ON the patch (off-patch samples get labeled as color and confuse KNN).\n")
    with open(TRAIN_FILE, "w") as f:
        for label in LABELS:
            input("Put the %s, then press Enter: " % prompts[label])
            time.sleep_ms(500)
            print("   sampling %s ... keep wiggling" % label)
            for _ in range(SAMPLES_PER_CLASS):
                for side, s_ in (("L", sL), ("R", sR)):
                    r, g, b, w = s_.read()
                    ch = _chroma((r, g, b))
                    if ch:
                        fr = refl(w, CAL[side][0], CAL[side][1])
                        f.write("%s,%.4f,%.4f,%.4f,%.3f,%s\n" % (side, ch[0], ch[1], ch[2], fr, label))
                time.sleep_ms(45)
    print("\nSaved training data to %s" % TRAIN_FILE)
    gc.collect()
    try:
        evaluate_knn()
    except MemoryError:
        print("\nOut of memory while evaluating (heap is fragmented after recording).")
        print("Your data IS saved. Set MODE = \"evaluate\" and run again for the results.")


def evaluate_knn():
    """Leave-one-out test on the SAVED training data (no re-recording needed)."""
    load_training()
    benchmark_knn()
    print("\nLeave-one-out results with the real accept rules (can take ~10-30 s with lots of samples).")
    print("(WHITE <-> LINE mix-ups don't matter: neither one triggers anything.)\n")
    for side in ("L", "R"):
        for lab in LABELS:
            tot = hit = low_votes = too_far = wrong = false_trig = 0
            mistaken, other, dsum, dn = {}, {}, 0.0, 0
            for idx, (a, b, cc, rf, l2) in enumerate(TRAIN[side]):
                if l2 != lab:
                    continue
                col, got, votes, md = _knn_core(a, b, cc, rf, side, idx)
                tot += 1
                if got == lab:
                    dsum += md
                    dn += 1
                if lab in COLORS:
                    if col == lab:
                        hit += 1
                    elif col is None:
                        if got == lab:
                            if votes < MIN_VOTES.get(lab, KNN_MIN_VOTES):
                                low_votes += 1
                            else:
                                too_far += 1
                        else:
                            mistaken[got] = mistaken.get(got, 0) + 1
                    else:
                        wrong += 1
                        other[col] = other.get(col, 0) + 1
                elif col is not None:
                    false_trig += 1
                    other[col] = other.get(col, 0) + 1
            avg_d = dsum / dn if dn else 0.0
            if lab in COLORS:
                print("  %s %-5s detected %2d/%d | rejected: votes %d, distance %d | mistaken for %s | wrong color %d %s | avg d %.3f" % (
                    side, lab, hit, tot, low_votes, too_far, mistaken if mistaken else "-", wrong, other if other else "", avg_d))
            else:
                print("  %s %-5s false triggers %d/%d %s" % (side, lab, false_trig, tot, other if other else ""))
        print()
    print("Goal: RED detected >= ~90%, false triggers on WHITE/LINE = 0.")
    print("votes -> lower MIN_VOTES for that color | distance -> raise KNN_MAX_DIST (compare 'avg d')")


# ---------- Buzzer / finish celebration on the car ----------
def tone(freq, ms):
    if BUZZER_PIN is None:
        time.sleep_ms(ms)
        return
    buzzer = PWM(Pin(BUZZER_PIN), freq=freq, duty_u16=32768)
    time.sleep_ms(ms)
    buzzer.deinit()


def maneuver_red():
    # Stay put: flash the NeoPixels red 4 times with a short beep each flash
    for _ in range(4):
        np_set(255, 0, 0)
        tone(880, 150)
        np_clear()
        time.sleep_ms(120)
    time.sleep_ms(100)


# ---------- Test modes ----------
def motor_test():
    print("Testing Motors: Left forward 1s, then Right forward 1s...")
    drive(0.35, 0)
    time.sleep(1)
    stop()
    time.sleep_ms(400)
    drive(0, 0.35)
    time.sleep(1)
    stop()


def sensor_view():
    sL, sR = make_sensors()
    load_training()
    benchmark_knn()
    print("\nStreaming live KNN data (Ctrl+C to stop)...")
    while True:
        rl, rr = sL.read(), sR.read()
        fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
        fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
        cL, labL, vL, dL = knn_classify(rl[:3], fl, "L")
        cR, labR, vR, dR = knn_classify(rr[:3], fr, "R")
        print("L: refl=%.2f %-5s v=%d d=%.3f [%s] | R: refl=%.2f %-5s v=%d d=%.3f [%s]" % (
            fl, labL, vL, dL, cL if cL else "---",
            fr, labR, vR, dR, cR if cR else "---"
        ))
        time.sleep_ms(150)


# ---------- BLE joystick driving ----------
buffer = b""
latest = None                    # most recent complete message line (bytes)


def callback(data):
    # Runs in IRQ context: only buffer and split. No motor, sensor or heavy work here.
    global buffer, latest
    buffer += data
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        latest = line


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def run():
    global latest
    from BLE_CEEO import Yell

    sL, sR = make_sensors()
    load_training()
    benchmark_knn()

    p = None
    try:
        p = Yell(NAME, interval_us=30000, verbose=False)
        p.callback = callback

        while True:
            stop()
            print("Waiting for the controller...")
            np_set(0, 0, 40)                     # blue = advertising
            if not p.connect_up():
                continue
            print("Connected")
            np_clear()

            last_msg = time.ticks_ms()
            ignore_until = time.ticks_ms()
            last_color = None
            hits = 0
            armed = True

            while p.is_connected:
                t0 = time.ticks_ms()

                # --- joystick message -> motors ---
                if latest is not None:
                    line, latest = latest, None
                    try:
                        parts = line.decode().split(",")
                        if parts[0] == "J" and len(parts) == 4:
                            x, y, boost = int(parts[1]), int(parts[2]), int(parts[3])
                            gain = BOOST_GAIN if boost else NORMAL_GAIN
                            l = clamp((y + x) / 100.0, -1.0, 1.0) * gain
                            r = clamp((y - x) / 100.0, -1.0, 1.0) * gain
                            drive(l, r)
                            last_msg = time.ticks_ms()
                    except (ValueError, UnicodeError):
                        pass                      # ignore a garbled message

                # --- failsafe ---
                if time.ticks_diff(time.ticks_ms(), last_msg) > FAILSAFE_MS:
                    stop()

                # --- finish line (red tape) via KNN ---
                rl, rr = sL.read(), sR.read()
                fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
                fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
                cL = knn_classify(rl[:3], fl, "L")[0]
                cR = knn_classify(rr[:3], fr, "R")[0]
                red = (cL == "RED") or (cR == "RED")

                if red and last_color == "RED":
                    hits += 1
                else:
                    hits = 1 if red else 0
                last_color = "RED" if red else None
                if not red:
                    armed = True

                now = time.ticks_ms()
                if red and armed and hits >= CONFIRM_RED and time.ticks_diff(now, ignore_until) > 0:
                    stop()
                    p.send("F\n")                 # tell the controller to celebrate
                    print("Finish line!")
                    maneuver_red()                # car's own flash + beep
                    armed = False
                    hits = 0
                    last_color = None
                    ignore_until = time.ticks_add(time.ticks_ms(), COOLDOWN_MS)
                    last_msg = time.ticks_ms()

                wait_loop(t0)

            print("Lost connection")
    finally:
        stop()
        np_clear()
        if p:
            p.disconnect()


if MODE == "calibrate":
    calibrate_knn()
elif MODE == "evaluate":
    evaluate_knn()
elif MODE == "sensors":
    sensor_view()
elif MODE == "motors":
    motor_test()
elif MODE == "run":
    run()# car.py -- MicroPython on ESP32 + Cytron ROBO-ESP32 (BLE peripheral / "Yeller")
# Joystick drive over BLE + KNN red-tape finish-line detection that messages the controller.
# Needs on the car's flash: BLE_CEEO.py and knn_train.csv (made by MODE = "calibrate").
#
# Messages:
#   controller -> car : "J,x,y,boost\n"   x,y in -100..100, boost 0/1
#   car -> controller : "F\n"             sent when the red finish line is detected

from machine import Pin, PWM, SoftI2C
import neopixel
import time
import math
import gc

# Options: "calibrate" -> record KNN training samples (saves knn_train.csv), then evaluate
#          "evaluate"  -> re-test saved samples with current settings
#          "sensors"   -> live KNN readout for tuning
#          "motors"    -> test wheel direction (both should roll forward)
#          "run"       -> BLE joystick driving + finish-line detection
MODE = "run"

# ======================= CONFIG =======================
# BLE
NAME = "Maria"                   # must match the controller; <= 8 chars. Make it unique to your team!
FAILSAFE_MS = 500                # no joystick message for this long -> stop the motors
NORMAL_GAIN = 0.6                # max motor speed (0..1) without boost
BOOST_GAIN = 1.0                 # max motor speed (0..1) with boost

# Motor pins: M1 (Right) = D12/D13, M2 (Left) = D14/D27
RIGHT_A, RIGHT_B = 12, 13
LEFT_A,  LEFT_B  = 14, 27
RIGHT_INVERT = True              # Inverted: M1B drives CW forward
LEFT_INVERT  = False             # M2A drives CCW forward

# Dual Grove Port assignments (both sensors use fixed I2C address 0x10)
L_SDA, L_SCL = 21, 22            # Grove 2 (Left Sensor)
R_SDA, R_SCL = 26, 25            # Grove 3 (Right Sensor)

# Cytron Robo-ESP32 onboard buzzer & NeoPixels
BUZZER_PIN = 23
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

LOOP_MS = 40                     # main loop period (VEML6040 40 ms integration)

# KNN color detection (unchanged from your v9)
KNN_K = 5
KNN_MIN_VOTES = 4
MIN_VOTES = {"RED": 4, "GREEN": 5, "BLUE": 5}
KNN_MAX_DIST = 0.12
BRIGHT_W = 0.10
SAMPLES_PER_CLASS = 60
RUNTIME_STRIDE = 1               # set 2 or 3 if the loop is too slow
TRAIN_FILE = "knn_train.csv"
SCALE = 10000
LABELS = ("WHITE", "LINE", "RED", "GREEN", "BLUE")
COLORS = ("RED", "GREEN", "BLUE")

CONFIRM_RED = 2                  # consecutive RED readings before the finish triggers
COOLDOWN_MS = 2000               # ignore the tape for this long after finishing

# Black/white brightness calibration (W channel): (w_black, w_white, ...)
CAL = {
    'L': (250.7, 397.0, (0.9532228, 0.84450968, 1.3041067)),
    'R': (285.1, 393.9, (0.92870264, 0.8217886, 1.4156862))
}
# ======================================================


# ---------- Motors ----------
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


# ---------- NeoPixels ----------
np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)


def np_set(r, g, b):
    for i in range(NUM_PIXELS):
        np[i] = (r, g, b)
    np.write()


def np_clear():
    np_set(0, 0, 0)


np_clear()


# ---------- VEML6040 Driver ----------
class VEML6040:
    ADDR = 0x10

    def __init__(self, i2c, it=0):  # 40ms integration
        self.i2c = i2c
        self.i2c.writeto_mem(self.ADDR, 0x00, bytes([(it << 4), 0x00]))

    def _rd(self, reg):
        d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
        return d[0] | (d[1] << 8)

    def read(self):
        return self._rd(0x08), self._rd(0x09), self._rd(0x0A), self._rd(0x0B)


def make_sensors():
    i2c_l = SoftI2C(sda=Pin(L_SDA), scl=Pin(L_SCL), freq=100000)
    i2c_r = SoftI2C(sda=Pin(R_SDA), scl=Pin(R_SCL), freq=100000)
    return VEML6040(i2c_l), VEML6040(i2c_r)


# ---------- Helpers ----------
def refl(w, black, white):
    if white <= black:
        return 0.0
    return max(0.0, min(1.0, (w - black) / (white - black)))


def _chroma(rgb):
    t = rgb[0] + rgb[1] + rgb[2]
    if t <= 0:
        return None
    return (rgb[0] / t, rgb[1] / t, rgb[2] / t)


def wait_loop(t0):
    """Sleep out the remainder of LOOP_MS so KNN compute time doesn't slow the loop."""
    rest = LOOP_MS - time.ticks_diff(time.ticks_ms(), t0)
    if rest > 0:
        time.sleep_ms(rest)


# ---------- KNN ----------
TRAIN = {"L": [], "R": []}
LABEL_MAP = {lab: lab for lab in LABELS}


def load_training():
    gc.collect()
    n = {"L": 0, "R": 0}
    try:
        f = open(TRAIN_FILE)
    except OSError:
        raise RuntimeError("No %s found. Run MODE = \"calibrate\" first." % TRAIN_FILE)
    with f:
        for line in f:
            if line[:1] in n:
                n[line[:1]] += 1
    rows = {"L": [None] * (n["L"] // RUNTIME_STRIDE), "R": [None] * (n["R"] // RUNTIME_STRIDE)}
    cnt = {"L": 0, "R": 0}
    filled = {"L": 0, "R": 0}

    with open(TRAIN_FILE) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) != 6 or p[0] not in rows or p[5] not in LABEL_MAP:
                continue
            side = p[0]
            cnt[side] += 1
            if cnt[side] % RUNTIME_STRIDE:
                continue
            k = filled[side]
            if k >= len(rows[side]):
                continue
            rows[side][k] = (int(float(p[1]) * SCALE), int(float(p[2]) * SCALE),
                             int(float(p[3]) * SCALE), int(float(p[4]) * BRIGHT_W * SCALE),
                             LABEL_MAP[p[5]])
            filled[side] = k + 1
    for side in ("L", "R"):
        if filled[side] < len(rows[side]):
            rows[side] = rows[side][:filled[side]]
    TRAIN["L"], TRAIN["R"] = rows["L"], rows["R"]
    gc.collect()
    print("Loaded KNN data: L=%d  R=%d samples (free RAM %d bytes)" % (len(TRAIN["L"]), len(TRAIN["R"]), gc.mem_free()))


def _knn_core(c0, c1, c2, q, side, skip=-1):
    """Integer KNN. Returns (color or None, winning_label, votes, mean_distance in chroma units)."""
    train = TRAIN[side]
    if not train:
        return None, "---", 0, 9.0
    k = KNN_K
    best = []
    worst = 0x3FFFFFFF
    i = -1
    for a, b, cc, rf, lab in train:
        i += 1
        if i == skip:
            continue
        d = c0 - a
        d2 = d * d
        if d2 >= worst:
            continue
        d = c1 - b
        d2 += d * d
        if d2 >= worst:
            continue
        d = c2 - cc
        d2 += d * d
        if d2 >= worst:
            continue
        d = q - rf
        d2 += d * d
        if d2 >= worst:
            continue
        if len(best) < k:
            best.append((d2, lab))
            if len(best) == k:
                best.sort()
                worst = best[-1][0]
        else:
            best[-1] = (d2, lab)
            best.sort()
            worst = best[-1][0]
    best.sort()

    counts = {}
    for d2, lab in best:
        counts[lab] = counts.get(lab, 0) + 1
    top = max(counts.values())
    winner = None
    for d2, lab in best:
        if counts[lab] == top:
            winner = lab
            break
    ds = [math.sqrt(d2) for d2, lab in best if lab == winner]
    mean_d = sum(ds) / len(ds) / SCALE

    if winner in COLORS and top >= MIN_VOTES.get(winner, KNN_MIN_VOTES) and mean_d < KNN_MAX_DIST:
        return winner, winner, top, mean_d
    return None, winner, top, mean_d


def knn_classify(rgb, w_refl, side):
    c = _chroma(rgb)
    if c is None:
        return None, "---", 0, 9.0
    return _knn_core(int(c[0] * SCALE), int(c[1] * SCALE), int(c[2] * SCALE),
                     int(w_refl * BRIGHT_W * SCALE), side)


def benchmark_knn():
    qs = {}
    for side in ("L", "R"):
        rows = [r for r in TRAIN[side] if r[4] == "LINE"] or TRAIN[side]
        qs[side] = rows[len(rows) // 2]
    n = 10
    t0 = time.ticks_us()
    for _ in range(n):
        for side in ("L", "R"):
            a, b, cc, rf, _lab = qs[side]
            _knn_core(a, b, cc, rf, side)
    ms = time.ticks_diff(time.ticks_us(), t0) / n / 1000
    print("KNN cost: %.1f ms per loop (both sensors), loop target %d ms" % (ms, LOOP_MS))
    if ms > LOOP_MS * 0.5:
        print("  That's heavy. Set RUNTIME_STRIDE = 2 (or 3) to use every Nth sample at runtime.")


def calibrate_knn():
    sL, sR = make_sensors()
    prompts = {
        "WHITE": "robot on bare WHITE floor (slide it around a little)",
        "LINE": "robot centered on the BLACK LINE; wiggle it ~1 cm left/right along the tape",
        "RED": "robot centered on the RED patch; only a FEW MM of wiggle, keep both sensors on it",
        "GREEN": "robot centered on the GREEN patch; only a FEW MM of wiggle, keep both sensors on it",
        "BLUE": "robot centered on the BLUE patch; only a FEW MM of wiggle, keep both sensors on it",
    }
    secs = SAMPLES_PER_CLASS * 0.045
    print("\nKeep the real height and orientation. Each class samples for ~%.1f s:" % secs)
    print("LINE / WHITE: move it around the way the robot will see it. Colors: only a few mm, so the")
    print("samples stay ON the patch (off-patch samples get labeled as color and confuse KNN).\n")
    with open(TRAIN_FILE, "w") as f:
        for label in LABELS:
            input("Put the %s, then press Enter: " % prompts[label])
            time.sleep_ms(500)
            print("   sampling %s ... keep wiggling" % label)
            for _ in range(SAMPLES_PER_CLASS):
                for side, s_ in (("L", sL), ("R", sR)):
                    r, g, b, w = s_.read()
                    ch = _chroma((r, g, b))
                    if ch:
                        fr = refl(w, CAL[side][0], CAL[side][1])
                        f.write("%s,%.4f,%.4f,%.4f,%.3f,%s\n" % (side, ch[0], ch[1], ch[2], fr, label))
                time.sleep_ms(45)
    print("\nSaved training data to %s" % TRAIN_FILE)
    gc.collect()
    try:
        evaluate_knn()
    except MemoryError:
        print("\nOut of memory while evaluating (heap is fragmented after recording).")
        print("Your data IS saved. Set MODE = \"evaluate\" and run again for the results.")


def evaluate_knn():
    """Leave-one-out test on the SAVED training data (no re-recording needed)."""
    load_training()
    benchmark_knn()
    print("\nLeave-one-out results with the real accept rules (can take ~10-30 s with lots of samples).")
    print("(WHITE <-> LINE mix-ups don't matter: neither one triggers anything.)\n")
    for side in ("L", "R"):
        for lab in LABELS:
            tot = hit = low_votes = too_far = wrong = false_trig = 0
            mistaken, other, dsum, dn = {}, {}, 0.0, 0
            for idx, (a, b, cc, rf, l2) in enumerate(TRAIN[side]):
                if l2 != lab:
                    continue
                col, got, votes, md = _knn_core(a, b, cc, rf, side, idx)
                tot += 1
                if got == lab:
                    dsum += md
                    dn += 1
                if lab in COLORS:
                    if col == lab:
                        hit += 1
                    elif col is None:
                        if got == lab:
                            if votes < MIN_VOTES.get(lab, KNN_MIN_VOTES):
                                low_votes += 1
                            else:
                                too_far += 1
                        else:
                            mistaken[got] = mistaken.get(got, 0) + 1
                    else:
                        wrong += 1
                        other[col] = other.get(col, 0) + 1
                elif col is not None:
                    false_trig += 1
                    other[col] = other.get(col, 0) + 1
            avg_d = dsum / dn if dn else 0.0
            if lab in COLORS:
                print("  %s %-5s detected %2d/%d | rejected: votes %d, distance %d | mistaken for %s | wrong color %d %s | avg d %.3f" % (
                    side, lab, hit, tot, low_votes, too_far, mistaken if mistaken else "-", wrong, other if other else "", avg_d))
            else:
                print("  %s %-5s false triggers %d/%d %s" % (side, lab, false_trig, tot, other if other else ""))
        print()
    print("Goal: RED detected >= ~90%, false triggers on WHITE/LINE = 0.")
    print("votes -> lower MIN_VOTES for that color | distance -> raise KNN_MAX_DIST (compare 'avg d')")


# ---------- Buzzer / finish celebration on the car ----------
def tone(freq, ms):
    if BUZZER_PIN is None:
        time.sleep_ms(ms)
        return
    buzzer = PWM(Pin(BUZZER_PIN), freq=freq, duty_u16=32768)
    time.sleep_ms(ms)
    buzzer.deinit()


def maneuver_red():
    # Stay put: flash the NeoPixels red 4 times with a short beep each flash
    for _ in range(4):
        np_set(255, 0, 0)
        tone(880, 150)
        np_clear()
        time.sleep_ms(120)
    time.sleep_ms(100)


# ---------- Test modes ----------
def motor_test():
    print("Testing Motors: Left forward 1s, then Right forward 1s...")
    drive(0.35, 0)
    time.sleep(1)
    stop()
    time.sleep_ms(400)
    drive(0, 0.35)
    time.sleep(1)
    stop()


def sensor_view():
    sL, sR = make_sensors()
    load_training()
    benchmark_knn()
    print("\nStreaming live KNN data (Ctrl+C to stop)...")
    while True:
        rl, rr = sL.read(), sR.read()
        fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
        fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
        cL, labL, vL, dL = knn_classify(rl[:3], fl, "L")
        cR, labR, vR, dR = knn_classify(rr[:3], fr, "R")
        print("L: refl=%.2f %-5s v=%d d=%.3f [%s] | R: refl=%.2f %-5s v=%d d=%.3f [%s]" % (
            fl, labL, vL, dL, cL if cL else "---",
            fr, labR, vR, dR, cR if cR else "---"
        ))
        time.sleep_ms(150)


# ---------- BLE joystick driving ----------
buffer = b""
latest = None                    # most recent complete message line (bytes)


def callback(data):
    # Runs in IRQ context: only buffer and split. No motor, sensor or heavy work here.
    global buffer, latest
    buffer += data
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        latest = line


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def run():
    global latest
    from BLE_CEEO import Yell

    sL, sR = make_sensors()
    load_training()
    benchmark_knn()

    p = None
    try:
        p = Yell(NAME, interval_us=30000, verbose=False)
        p.callback = callback

        while True:
            stop()
            print("Waiting for the controller...")
            np_set(0, 0, 40)                     # blue = advertising
            if not p.connect_up():
                continue
            print("Connected")
            np_clear()

            last_msg = time.ticks_ms()
            ignore_until = time.ticks_ms()
            last_color = None
            hits = 0
            armed = True

            while p.is_connected:
                t0 = time.ticks_ms()

                # --- joystick message -> motors ---
                if latest is not None:
                    line, latest = latest, None
                    try:
                        parts = line.decode().split(",")
                        if parts[0] == "J" and len(parts) == 4:
                            x, y, boost = int(parts[1]), int(parts[2]), int(parts[3])
                            gain = BOOST_GAIN if boost else NORMAL_GAIN
                            l = clamp((y + x) / 100.0, -1.0, 1.0) * gain
                            r = clamp((y - x) / 100.0, -1.0, 1.0) * gain
                            drive(l, r)
                            last_msg = time.ticks_ms()
                    except (ValueError, UnicodeError):
                        pass                      # ignore a garbled message

                # --- failsafe ---
                if time.ticks_diff(time.ticks_ms(), last_msg) > FAILSAFE_MS:
                    stop()

                # --- finish line (red tape) via KNN ---
                rl, rr = sL.read(), sR.read()
                fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
                fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
                cL = knn_classify(rl[:3], fl, "L")[0]
                cR = knn_classify(rr[:3], fr, "R")[0]
                red = (cL == "RED") or (cR == "RED")

                if red and last_color == "RED":
                    hits += 1
                else:
                    hits = 1 if red else 0
                last_color = "RED" if red else None
                if not red:
                    armed = True

                now = time.ticks_ms()
                if red and armed and hits >= CONFIRM_RED and time.ticks_diff(now, ignore_until) > 0:
                    stop()
                    p.send("F\n")                 # tell the controller to celebrate
                    print("Finish line!")
                    maneuver_red()                # car's own flash + beep
                    armed = False
                    hits = 0
                    last_color = None
                    ignore_until = time.ticks_add(time.ticks_ms(), COOLDOWN_MS)
                    last_msg = time.ticks_ms()

                wait_loop(t0)

            print("Lost connection")
    finally:
        stop()
        np_clear()
        if p:
            p.disconnect()


if MODE == "calibrate":
    calibrate_knn()
elif MODE == "evaluate":
    evaluate_knn()
elif MODE == "sensors":
    sensor_view()
elif MODE == "motors":
    motor_test()
elif MODE == "run":
    run()# car.py -- MicroPython on ESP32 + Cytron ROBO-ESP32 (BLE peripheral / "Yeller")
# Joystick drive over BLE + KNN red-tape finish-line detection that messages the controller.
# Needs on the car's flash: BLE_CEEO.py and knn_train.csv (made by MODE = "calibrate").
#
# Messages:
#   controller -> car : "J,x,y,boost\n"   x,y in -100..100, boost 0/1
#   car -> controller : "F\n"             sent when the red finish line is detected

from machine import Pin, PWM, SoftI2C
import neopixel
import time
import math
import gc

# Options: "calibrate" -> record KNN training samples (saves knn_train.csv), then evaluate
#          "evaluate"  -> re-test saved samples with current settings
#          "sensors"   -> live KNN readout for tuning
#          "motors"    -> test wheel direction (both should roll forward)
#          "run"       -> BLE joystick driving + finish-line detection
MODE = "run"

# ======================= CONFIG =======================
# BLE
NAME = "Maria"                   # must match the controller; <= 8 chars. Make it unique to your team!
FAILSAFE_MS = 500                # no joystick message for this long -> stop the motors
NORMAL_GAIN = 0.6                # max motor speed (0..1) without boost
BOOST_GAIN = 1.0                 # max motor speed (0..1) with boost

# Motor pins: M1 (Right) = D12/D13, M2 (Left) = D14/D27
RIGHT_A, RIGHT_B = 12, 13
LEFT_A,  LEFT_B  = 14, 27
RIGHT_INVERT = True              # Inverted: M1B drives CW forward
LEFT_INVERT  = False             # M2A drives CCW forward

# Dual Grove Port assignments (both sensors use fixed I2C address 0x10)
L_SDA, L_SCL = 21, 22            # Grove 2 (Left Sensor)
R_SDA, R_SCL = 26, 25            # Grove 3 (Right Sensor)

# Cytron Robo-ESP32 onboard buzzer & NeoPixels
BUZZER_PIN = 23
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

LOOP_MS = 40                     # main loop period (VEML6040 40 ms integration)

# KNN color detection (unchanged from your v9)
KNN_K = 5
KNN_MIN_VOTES = 4
MIN_VOTES = {"RED": 4, "GREEN": 5, "BLUE": 5}
KNN_MAX_DIST = 0.12
BRIGHT_W = 0.10
SAMPLES_PER_CLASS = 60
RUNTIME_STRIDE = 1               # set 2 or 3 if the loop is too slow
TRAIN_FILE = "knn_train.csv"
SCALE = 10000
LABELS = ("WHITE", "LINE", "RED", "GREEN", "BLUE")
COLORS = ("RED", "GREEN", "BLUE")

CONFIRM_RED = 2                  # consecutive RED readings before the finish triggers
COOLDOWN_MS = 2000               # ignore the tape for this long after finishing

# Black/white brightness calibration (W channel): (w_black, w_white, ...)
CAL = {
    'L': (250.7, 397.0, (0.9532228, 0.84450968, 1.3041067)),
    'R': (285.1, 393.9, (0.92870264, 0.8217886, 1.4156862))
}
# ======================================================


# ---------- Motors ----------
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


# ---------- NeoPixels ----------
np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)


def np_set(r, g, b):
    for i in range(NUM_PIXELS):
        np[i] = (r, g, b)
    np.write()


def np_clear():
    np_set(0, 0, 0)


np_clear()


# ---------- VEML6040 Driver ----------
class VEML6040:
    ADDR = 0x10

    def __init__(self, i2c, it=0):  # 40ms integration
        self.i2c = i2c
        self.i2c.writeto_mem(self.ADDR, 0x00, bytes([(it << 4), 0x00]))

    def _rd(self, reg):
        d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
        return d[0] | (d[1] << 8)

    def read(self):
        return self._rd(0x08), self._rd(0x09), self._rd(0x0A), self._rd(0x0B)


def make_sensors():
    i2c_l = SoftI2C(sda=Pin(L_SDA), scl=Pin(L_SCL), freq=100000)
    i2c_r = SoftI2C(sda=Pin(R_SDA), scl=Pin(R_SCL), freq=100000)
    return VEML6040(i2c_l), VEML6040(i2c_r)


# ---------- Helpers ----------
def refl(w, black, white):
    if white <= black:
        return 0.0
    return max(0.0, min(1.0, (w - black) / (white - black)))


def _chroma(rgb):
    t = rgb[0] + rgb[1] + rgb[2]
    if t <= 0:
        return None
    return (rgb[0] / t, rgb[1] / t, rgb[2] / t)


def wait_loop(t0):
    """Sleep out the remainder of LOOP_MS so KNN compute time doesn't slow the loop."""
    rest = LOOP_MS - time.ticks_diff(time.ticks_ms(), t0)
    if rest > 0:
        time.sleep_ms(rest)


# ---------- KNN ----------
TRAIN = {"L": [], "R": []}
LABEL_MAP = {lab: lab for lab in LABELS}


def load_training():
    gc.collect()
    n = {"L": 0, "R": 0}
    try:
        f = open(TRAIN_FILE)
    except OSError:
        raise RuntimeError("No %s found. Run MODE = \"calibrate\" first." % TRAIN_FILE)
    with f:
        for line in f:
            if line[:1] in n:
                n[line[:1]] += 1
    rows = {"L": [None] * (n["L"] // RUNTIME_STRIDE), "R": [None] * (n["R"] // RUNTIME_STRIDE)}
    cnt = {"L": 0, "R": 0}
    filled = {"L": 0, "R": 0}

    with open(TRAIN_FILE) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) != 6 or p[0] not in rows or p[5] not in LABEL_MAP:
                continue
            side = p[0]
            cnt[side] += 1
            if cnt[side] % RUNTIME_STRIDE:
                continue
            k = filled[side]
            if k >= len(rows[side]):
                continue
            rows[side][k] = (int(float(p[1]) * SCALE), int(float(p[2]) * SCALE),
                             int(float(p[3]) * SCALE), int(float(p[4]) * BRIGHT_W * SCALE),
                             LABEL_MAP[p[5]])
            filled[side] = k + 1
    for side in ("L", "R"):
        if filled[side] < len(rows[side]):
            rows[side] = rows[side][:filled[side]]
    TRAIN["L"], TRAIN["R"] = rows["L"], rows["R"]
    gc.collect()
    print("Loaded KNN data: L=%d  R=%d samples (free RAM %d bytes)" % (len(TRAIN["L"]), len(TRAIN["R"]), gc.mem_free()))


def _knn_core(c0, c1, c2, q, side, skip=-1):
    """Integer KNN. Returns (color or None, winning_label, votes, mean_distance in chroma units)."""
    train = TRAIN[side]
    if not train:
        return None, "---", 0, 9.0
    k = KNN_K
    best = []
    worst = 0x3FFFFFFF
    i = -1
    for a, b, cc, rf, lab in train:
        i += 1
        if i == skip:
            continue
        d = c0 - a
        d2 = d * d
        if d2 >= worst:
            continue
        d = c1 - b
        d2 += d * d
        if d2 >= worst:
            continue
        d = c2 - cc
        d2 += d * d
        if d2 >= worst:
            continue
        d = q - rf
        d2 += d * d
        if d2 >= worst:
            continue
        if len(best) < k:
            best.append((d2, lab))
            if len(best) == k:
                best.sort()
                worst = best[-1][0]
        else:
            best[-1] = (d2, lab)
            best.sort()
            worst = best[-1][0]
    best.sort()

    counts = {}
    for d2, lab in best:
        counts[lab] = counts.get(lab, 0) + 1
    top = max(counts.values())
    winner = None
    for d2, lab in best:
        if counts[lab] == top:
            winner = lab
            break
    ds = [math.sqrt(d2) for d2, lab in best if lab == winner]
    mean_d = sum(ds) / len(ds) / SCALE

    if winner in COLORS and top >= MIN_VOTES.get(winner, KNN_MIN_VOTES) and mean_d < KNN_MAX_DIST:
        return winner, winner, top, mean_d
    return None, winner, top, mean_d


def knn_classify(rgb, w_refl, side):
    c = _chroma(rgb)
    if c is None:
        return None, "---", 0, 9.0
    return _knn_core(int(c[0] * SCALE), int(c[1] * SCALE), int(c[2] * SCALE),
                     int(w_refl * BRIGHT_W * SCALE), side)


def benchmark_knn():
    qs = {}
    for side in ("L", "R"):
        rows = [r for r in TRAIN[side] if r[4] == "LINE"] or TRAIN[side]
        qs[side] = rows[len(rows) // 2]
    n = 10
    t0 = time.ticks_us()
    for _ in range(n):
        for side in ("L", "R"):
            a, b, cc, rf, _lab = qs[side]
            _knn_core(a, b, cc, rf, side)
    ms = time.ticks_diff(time.ticks_us(), t0) / n / 1000
    print("KNN cost: %.1f ms per loop (both sensors), loop target %d ms" % (ms, LOOP_MS))
    if ms > LOOP_MS * 0.5:
        print("  That's heavy. Set RUNTIME_STRIDE = 2 (or 3) to use every Nth sample at runtime.")


def calibrate_knn():
    sL, sR = make_sensors()
    prompts = {
        "WHITE": "robot on bare WHITE floor (slide it around a little)",
        "LINE": "robot centered on the BLACK LINE; wiggle it ~1 cm left/right along the tape",
        "RED": "robot centered on the RED patch; only a FEW MM of wiggle, keep both sensors on it",
        "GREEN": "robot centered on the GREEN patch; only a FEW MM of wiggle, keep both sensors on it",
        "BLUE": "robot centered on the BLUE patch; only a FEW MM of wiggle, keep both sensors on it",
    }
    secs = SAMPLES_PER_CLASS * 0.045
    print("\nKeep the real height and orientation. Each class samples for ~%.1f s:" % secs)
    print("LINE / WHITE: move it around the way the robot will see it. Colors: only a few mm, so the")
    print("samples stay ON the patch (off-patch samples get labeled as color and confuse KNN).\n")
    with open(TRAIN_FILE, "w") as f:
        for label in LABELS:
            input("Put the %s, then press Enter: " % prompts[label])
            time.sleep_ms(500)
            print("   sampling %s ... keep wiggling" % label)
            for _ in range(SAMPLES_PER_CLASS):
                for side, s_ in (("L", sL), ("R", sR)):
                    r, g, b, w = s_.read()
                    ch = _chroma((r, g, b))
                    if ch:
                        fr = refl(w, CAL[side][0], CAL[side][1])
                        f.write("%s,%.4f,%.4f,%.4f,%.3f,%s\n" % (side, ch[0], ch[1], ch[2], fr, label))
                time.sleep_ms(45)
    print("\nSaved training data to %s" % TRAIN_FILE)
    gc.collect()
    try:
        evaluate_knn()
    except MemoryError:
        print("\nOut of memory while evaluating (heap is fragmented after recording).")
        print("Your data IS saved. Set MODE = \"evaluate\" and run again for the results.")


def evaluate_knn():
    """Leave-one-out test on the SAVED training data (no re-recording needed)."""
    load_training()
    benchmark_knn()
    print("\nLeave-one-out results with the real accept rules (can take ~10-30 s with lots of samples).")
    print("(WHITE <-> LINE mix-ups don't matter: neither one triggers anything.)\n")
    for side in ("L", "R"):
        for lab in LABELS:
            tot = hit = low_votes = too_far = wrong = false_trig = 0
            mistaken, other, dsum, dn = {}, {}, 0.0, 0
            for idx, (a, b, cc, rf, l2) in enumerate(TRAIN[side]):
                if l2 != lab:
                    continue
                col, got, votes, md = _knn_core(a, b, cc, rf, side, idx)
                tot += 1
                if got == lab:
                    dsum += md
                    dn += 1
                if lab in COLORS:
                    if col == lab:
                        hit += 1
                    elif col is None:
                        if got == lab:
                            if votes < MIN_VOTES.get(lab, KNN_MIN_VOTES):
                                low_votes += 1
                            else:
                                too_far += 1
                        else:
                            mistaken[got] = mistaken.get(got, 0) + 1
                    else:
                        wrong += 1
                        other[col] = other.get(col, 0) + 1
                elif col is not None:
                    false_trig += 1
                    other[col] = other.get(col, 0) + 1
            avg_d = dsum / dn if dn else 0.0
            if lab in COLORS:
                print("  %s %-5s detected %2d/%d | rejected: votes %d, distance %d | mistaken for %s | wrong color %d %s | avg d %.3f" % (
                    side, lab, hit, tot, low_votes, too_far, mistaken if mistaken else "-", wrong, other if other else "", avg_d))
            else:
                print("  %s %-5s false triggers %d/%d %s" % (side, lab, false_trig, tot, other if other else ""))
        print()
    print("Goal: RED detected >= ~90%, false triggers on WHITE/LINE = 0.")
    print("votes -> lower MIN_VOTES for that color | distance -> raise KNN_MAX_DIST (compare 'avg d')")


# ---------- Buzzer / finish celebration on the car ----------
def tone(freq, ms):
    if BUZZER_PIN is None:
        time.sleep_ms(ms)
        return
    buzzer = PWM(Pin(BUZZER_PIN), freq=freq, duty_u16=32768)
    time.sleep_ms(ms)
    buzzer.deinit()


def maneuver_red():
    # Stay put: flash the NeoPixels red 4 times with a short beep each flash
    for _ in range(4):
        np_set(255, 0, 0)
        tone(880, 150)
        np_clear()
        time.sleep_ms(120)
    time.sleep_ms(100)


# ---------- Test modes ----------
def motor_test():
    print("Testing Motors: Left forward 1s, then Right forward 1s...")
    drive(0.35, 0)
    time.sleep(1)
    stop()
    time.sleep_ms(400)
    drive(0, 0.35)
    time.sleep(1)
    stop()


def sensor_view():
    sL, sR = make_sensors()
    load_training()
    benchmark_knn()
    print("\nStreaming live KNN data (Ctrl+C to stop)...")
    while True:
        rl, rr = sL.read(), sR.read()
        fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
        fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
        cL, labL, vL, dL = knn_classify(rl[:3], fl, "L")
        cR, labR, vR, dR = knn_classify(rr[:3], fr, "R")
        print("L: refl=%.2f %-5s v=%d d=%.3f [%s] | R: refl=%.2f %-5s v=%d d=%.3f [%s]" % (
            fl, labL, vL, dL, cL if cL else "---",
            fr, labR, vR, dR, cR if cR else "---"
        ))
        time.sleep_ms(150)


# ---------- BLE joystick driving ----------
buffer = b""
latest = None                    # most recent complete message line (bytes)


def callback(data):
    # Runs in IRQ context: only buffer and split. No motor, sensor or heavy work here.
    global buffer, latest
    buffer += data
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        latest = line


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def run():
    global latest
    from BLE_CEEO import Yell

    sL, sR = make_sensors()
    load_training()
    benchmark_knn()

    p = None
    try:
        p = Yell(NAME, interval_us=30000, verbose=False)
        p.callback = callback

        while True:
            stop()
            print("Waiting for the controller...")
            np_set(0, 0, 40)                     # blue = advertising
            if not p.connect_up():
                continue
            print("Connected")
            np_clear()

            last_msg = time.ticks_ms()
            ignore_until = time.ticks_ms()
            last_color = None
            hits = 0
            armed = True

            while p.is_connected:
                t0 = time.ticks_ms()

                # --- joystick message -> motors ---
                if latest is not None:
                    line, latest = latest, None
                    try:
                        parts = line.decode().split(",")
                        if parts[0] == "J" and len(parts) == 4:
                            x, y, boost = int(parts[1]), int(parts[2]), int(parts[3])
                            gain = BOOST_GAIN if boost else NORMAL_GAIN
                            l = clamp((y + x) / 100.0, -1.0, 1.0) * gain
                            r = clamp((y - x) / 100.0, -1.0, 1.0) * gain
                            drive(l, r)
                            last_msg = time.ticks_ms()
                    except (ValueError, UnicodeError):
                        pass                      # ignore a garbled message

                # --- failsafe ---
                if time.ticks_diff(time.ticks_ms(), last_msg) > FAILSAFE_MS:
                    stop()

                # --- finish line (red tape) via KNN ---
                rl, rr = sL.read(), sR.read()
                fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
                fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
                cL = knn_classify(rl[:3], fl, "L")[0]
                cR = knn_classify(rr[:3], fr, "R")[0]
                red = (cL == "RED") or (cR == "RED")

                if red and last_color == "RED":
                    hits += 1
                else:
                    hits = 1 if red else 0
                last_color = "RED" if red else None
                if not red:
                    armed = True

                now = time.ticks_ms()
                if red and armed and hits >= CONFIRM_RED and time.ticks_diff(now, ignore_until) > 0:
                    stop()
                    p.send("F\n")                 # tell the controller to celebrate
                    print("Finish line!")
                    maneuver_red()                # car's own flash + beep
                    armed = False
                    hits = 0
                    last_color = None
                    ignore_until = time.ticks_add(time.ticks_ms(), COOLDOWN_MS)
                    last_msg = time.ticks_ms()

                wait_loop(t0)

            print("Lost connection")
    finally:
        stop()
        np_clear()
        if p:
            p.disconnect()


if MODE == "calibrate":
    calibrate_knn()
elif MODE == "evaluate":
    evaluate_knn()
elif MODE == "sensors":
    sensor_view()
elif MODE == "motors":
    motor_test()
elif MODE == "run":
    run()# car.py -- MicroPython on ESP32 + Cytron ROBO-ESP32 (BLE peripheral / "Yeller")
# Joystick drive over BLE + KNN red-tape finish-line detection that messages the controller.
# Needs on the car's flash: BLE_CEEO.py and knn_train.csv (made by MODE = "calibrate").
#
# Messages:
#   controller -> car : "J,x,y,boost\n"   x,y in -100..100, boost 0/1
#   car -> controller : "F\n"             sent when the red finish line is detected

from machine import Pin, PWM, SoftI2C
import neopixel
import time
import math
import gc

# Options: "calibrate" -> record KNN training samples (saves knn_train.csv), then evaluate
#          "evaluate"  -> re-test saved samples with current settings
#          "sensors"   -> live KNN readout for tuning
#          "motors"    -> test wheel direction (both should roll forward)
#          "run"       -> BLE joystick driving + finish-line detection
MODE = "run"

# ======================= CONFIG =======================
# BLE
NAME = "Maria"                   # must match the controller; <= 8 chars. Make it unique to your team!
FAILSAFE_MS = 500                # no joystick message for this long -> stop the motors
NORMAL_GAIN = 0.6                # max motor speed (0..1) without boost
BOOST_GAIN = 1.0                 # max motor speed (0..1) with boost

# Motor pins: M1 (Right) = D12/D13, M2 (Left) = D14/D27
RIGHT_A, RIGHT_B = 12, 13
LEFT_A,  LEFT_B  = 14, 27
RIGHT_INVERT = True              # Inverted: M1B drives CW forward
LEFT_INVERT  = False             # M2A drives CCW forward

# Dual Grove Port assignments (both sensors use fixed I2C address 0x10)
L_SDA, L_SCL = 21, 22            # Grove 2 (Left Sensor)
R_SDA, R_SCL = 26, 25            # Grove 3 (Right Sensor)

# Cytron Robo-ESP32 onboard buzzer & NeoPixels
BUZZER_PIN = 23
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

LOOP_MS = 40                     # main loop period (VEML6040 40 ms integration)

# KNN color detection (unchanged from your v9)
KNN_K = 5
KNN_MIN_VOTES = 4
MIN_VOTES = {"RED": 4, "GREEN": 5, "BLUE": 5}
KNN_MAX_DIST = 0.12
BRIGHT_W = 0.10
SAMPLES_PER_CLASS = 60
RUNTIME_STRIDE = 1               # set 2 or 3 if the loop is too slow
TRAIN_FILE = "knn_train.csv"
SCALE = 10000
LABELS = ("WHITE", "LINE", "RED", "GREEN", "BLUE")
COLORS = ("RED", "GREEN", "BLUE")

CONFIRM_RED = 2                  # consecutive RED readings before the finish triggers
COOLDOWN_MS = 2000               # ignore the tape for this long after finishing

# Black/white brightness calibration (W channel): (w_black, w_white, ...)
CAL = {
    'L': (250.7, 397.0, (0.9532228, 0.84450968, 1.3041067)),
    'R': (285.1, 393.9, (0.92870264, 0.8217886, 1.4156862))
}
# ======================================================


# ---------- Motors ----------
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


# ---------- NeoPixels ----------
np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)


def np_set(r, g, b):
    for i in range(NUM_PIXELS):
        np[i] = (r, g, b)
    np.write()


def np_clear():
    np_set(0, 0, 0)


np_clear()


# ---------- VEML6040 Driver ----------
class VEML6040:
    ADDR = 0x10

    def __init__(self, i2c, it=0):  # 40ms integration
        self.i2c = i2c
        self.i2c.writeto_mem(self.ADDR, 0x00, bytes([(it << 4), 0x00]))

    def _rd(self, reg):
        d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
        return d[0] | (d[1] << 8)

    def read(self):
        return self._rd(0x08), self._rd(0x09), self._rd(0x0A), self._rd(0x0B)


def make_sensors():
    i2c_l = SoftI2C(sda=Pin(L_SDA), scl=Pin(L_SCL), freq=100000)
    i2c_r = SoftI2C(sda=Pin(R_SDA), scl=Pin(R_SCL), freq=100000)
    return VEML6040(i2c_l), VEML6040(i2c_r)


# ---------- Helpers ----------
def refl(w, black, white):
    if white <= black:
        return 0.0
    return max(0.0, min(1.0, (w - black) / (white - black)))


def _chroma(rgb):
    t = rgb[0] + rgb[1] + rgb[2]
    if t <= 0:
        return None
    return (rgb[0] / t, rgb[1] / t, rgb[2] / t)


def wait_loop(t0):
    """Sleep out the remainder of LOOP_MS so KNN compute time doesn't slow the loop."""
    rest = LOOP_MS - time.ticks_diff(time.ticks_ms(), t0)
    if rest > 0:
        time.sleep_ms(rest)


# ---------- KNN ----------
TRAIN = {"L": [], "R": []}
LABEL_MAP = {lab: lab for lab in LABELS}


def load_training():
    gc.collect()
    n = {"L": 0, "R": 0}
    try:
        f = open(TRAIN_FILE)
    except OSError:
        raise RuntimeError("No %s found. Run MODE = \"calibrate\" first." % TRAIN_FILE)
    with f:
        for line in f:
            if line[:1] in n:
                n[line[:1]] += 1
    rows = {"L": [None] * (n["L"] // RUNTIME_STRIDE), "R": [None] * (n["R"] // RUNTIME_STRIDE)}
    cnt = {"L": 0, "R": 0}
    filled = {"L": 0, "R": 0}

    with open(TRAIN_FILE) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) != 6 or p[0] not in rows or p[5] not in LABEL_MAP:
                continue
            side = p[0]
            cnt[side] += 1
            if cnt[side] % RUNTIME_STRIDE:
                continue
            k = filled[side]
            if k >= len(rows[side]):
                continue
            rows[side][k] = (int(float(p[1]) * SCALE), int(float(p[2]) * SCALE),
                             int(float(p[3]) * SCALE), int(float(p[4]) * BRIGHT_W * SCALE),
                             LABEL_MAP[p[5]])
            filled[side] = k + 1
    for side in ("L", "R"):
        if filled[side] < len(rows[side]):
            rows[side] = rows[side][:filled[side]]
    TRAIN["L"], TRAIN["R"] = rows["L"], rows["R"]
    gc.collect()
    print("Loaded KNN data: L=%d  R=%d samples (free RAM %d bytes)" % (len(TRAIN["L"]), len(TRAIN["R"]), gc.mem_free()))


def _knn_core(c0, c1, c2, q, side, skip=-1):
    """Integer KNN. Returns (color or None, winning_label, votes, mean_distance in chroma units)."""
    train = TRAIN[side]
    if not train:
        return None, "---", 0, 9.0
    k = KNN_K
    best = []
    worst = 0x3FFFFFFF
    i = -1
    for a, b, cc, rf, lab in train:
        i += 1
        if i == skip:
            continue
        d = c0 - a
        d2 = d * d
        if d2 >= worst:
            continue
        d = c1 - b
        d2 += d * d
        if d2 >= worst:
            continue
        d = c2 - cc
        d2 += d * d
        if d2 >= worst:
            continue
        d = q - rf
        d2 += d * d
        if d2 >= worst:
            continue
        if len(best) < k:
            best.append((d2, lab))
            if len(best) == k:
                best.sort()
                worst = best[-1][0]
        else:
            best[-1] = (d2, lab)
            best.sort()
            worst = best[-1][0]
    best.sort()

    counts = {}
    for d2, lab in best:
        counts[lab] = counts.get(lab, 0) + 1
    top = max(counts.values())
    winner = None
    for d2, lab in best:
        if counts[lab] == top:
            winner = lab
            break
    ds = [math.sqrt(d2) for d2, lab in best if lab == winner]
    mean_d = sum(ds) / len(ds) / SCALE

    if winner in COLORS and top >= MIN_VOTES.get(winner, KNN_MIN_VOTES) and mean_d < KNN_MAX_DIST:
        return winner, winner, top, mean_d
    return None, winner, top, mean_d


def knn_classify(rgb, w_refl, side):
    c = _chroma(rgb)
    if c is None:
        return None, "---", 0, 9.0
    return _knn_core(int(c[0] * SCALE), int(c[1] * SCALE), int(c[2] * SCALE),
                     int(w_refl * BRIGHT_W * SCALE), side)


def benchmark_knn():
    qs = {}
    for side in ("L", "R"):
        rows = [r for r in TRAIN[side] if r[4] == "LINE"] or TRAIN[side]
        qs[side] = rows[len(rows) // 2]
    n = 10
    t0 = time.ticks_us()
    for _ in range(n):
        for side in ("L", "R"):
            a, b, cc, rf, _lab = qs[side]
            _knn_core(a, b, cc, rf, side)
    ms = time.ticks_diff(time.ticks_us(), t0) / n / 1000
    print("KNN cost: %.1f ms per loop (both sensors), loop target %d ms" % (ms, LOOP_MS))
    if ms > LOOP_MS * 0.5:
        print("  That's heavy. Set RUNTIME_STRIDE = 2 (or 3) to use every Nth sample at runtime.")


def calibrate_knn():
    sL, sR = make_sensors()
    prompts = {
        "WHITE": "robot on bare WHITE floor (slide it around a little)",
        "LINE": "robot centered on the BLACK LINE; wiggle it ~1 cm left/right along the tape",
        "RED": "robot centered on the RED patch; only a FEW MM of wiggle, keep both sensors on it",
        "GREEN": "robot centered on the GREEN patch; only a FEW MM of wiggle, keep both sensors on it",
        "BLUE": "robot centered on the BLUE patch; only a FEW MM of wiggle, keep both sensors on it",
    }
    secs = SAMPLES_PER_CLASS * 0.045
    print("\nKeep the real height and orientation. Each class samples for ~%.1f s:" % secs)
    print("LINE / WHITE: move it around the way the robot will see it. Colors: only a few mm, so the")
    print("samples stay ON the patch (off-patch samples get labeled as color and confuse KNN).\n")
    with open(TRAIN_FILE, "w") as f:
        for label in LABELS:
            input("Put the %s, then press Enter: " % prompts[label])
            time.sleep_ms(500)
            print("   sampling %s ... keep wiggling" % label)
            for _ in range(SAMPLES_PER_CLASS):
                for side, s_ in (("L", sL), ("R", sR)):
                    r, g, b, w = s_.read()
                    ch = _chroma((r, g, b))
                    if ch:
                        fr = refl(w, CAL[side][0], CAL[side][1])
                        f.write("%s,%.4f,%.4f,%.4f,%.3f,%s\n" % (side, ch[0], ch[1], ch[2], fr, label))
                time.sleep_ms(45)
    print("\nSaved training data to %s" % TRAIN_FILE)
    gc.collect()
    try:
        evaluate_knn()
    except MemoryError:
        print("\nOut of memory while evaluating (heap is fragmented after recording).")
        print("Your data IS saved. Set MODE = \"evaluate\" and run again for the results.")


def evaluate_knn():
    """Leave-one-out test on the SAVED training data (no re-recording needed)."""
    load_training()
    benchmark_knn()
    print("\nLeave-one-out results with the real accept rules (can take ~10-30 s with lots of samples).")
    print("(WHITE <-> LINE mix-ups don't matter: neither one triggers anything.)\n")
    for side in ("L", "R"):
        for lab in LABELS:
            tot = hit = low_votes = too_far = wrong = false_trig = 0
            mistaken, other, dsum, dn = {}, {}, 0.0, 0
            for idx, (a, b, cc, rf, l2) in enumerate(TRAIN[side]):
                if l2 != lab:
                    continue
                col, got, votes, md = _knn_core(a, b, cc, rf, side, idx)
                tot += 1
                if got == lab:
                    dsum += md
                    dn += 1
                if lab in COLORS:
                    if col == lab:
                        hit += 1
                    elif col is None:
                        if got == lab:
                            if votes < MIN_VOTES.get(lab, KNN_MIN_VOTES):
                                low_votes += 1
                            else:
                                too_far += 1
                        else:
                            mistaken[got] = mistaken.get(got, 0) + 1
                    else:
                        wrong += 1
                        other[col] = other.get(col, 0) + 1
                elif col is not None:
                    false_trig += 1
                    other[col] = other.get(col, 0) + 1
            avg_d = dsum / dn if dn else 0.0
            if lab in COLORS:
                print("  %s %-5s detected %2d/%d | rejected: votes %d, distance %d | mistaken for %s | wrong color %d %s | avg d %.3f" % (
                    side, lab, hit, tot, low_votes, too_far, mistaken if mistaken else "-", wrong, other if other else "", avg_d))
            else:
                print("  %s %-5s false triggers %d/%d %s" % (side, lab, false_trig, tot, other if other else ""))
        print()
    print("Goal: RED detected >= ~90%, false triggers on WHITE/LINE = 0.")
    print("votes -> lower MIN_VOTES for that color | distance -> raise KNN_MAX_DIST (compare 'avg d')")


# ---------- Buzzer / finish celebration on the car ----------
def tone(freq, ms):
    if BUZZER_PIN is None:
        time.sleep_ms(ms)
        return
    buzzer = PWM(Pin(BUZZER_PIN), freq=freq, duty_u16=32768)
    time.sleep_ms(ms)
    buzzer.deinit()


def maneuver_red():
    # Stay put: flash the NeoPixels red 4 times with a short beep each flash
    for _ in range(4):
        np_set(255, 0, 0)
        tone(880, 150)
        np_clear()
        time.sleep_ms(120)
    time.sleep_ms(100)


# ---------- Test modes ----------
def motor_test():
    print("Testing Motors: Left forward 1s, then Right forward 1s...")
    drive(0.35, 0)
    time.sleep(1)
    stop()
    time.sleep_ms(400)
    drive(0, 0.35)
    time.sleep(1)
    stop()


def sensor_view():
    sL, sR = make_sensors()
    load_training()
    benchmark_knn()
    print("\nStreaming live KNN data (Ctrl+C to stop)...")
    while True:
        rl, rr = sL.read(), sR.read()
        fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
        fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
        cL, labL, vL, dL = knn_classify(rl[:3], fl, "L")
        cR, labR, vR, dR = knn_classify(rr[:3], fr, "R")
        print("L: refl=%.2f %-5s v=%d d=%.3f [%s] | R: refl=%.2f %-5s v=%d d=%.3f [%s]" % (
            fl, labL, vL, dL, cL if cL else "---",
            fr, labR, vR, dR, cR if cR else "---"
        ))
        time.sleep_ms(150)


# ---------- BLE joystick driving ----------
buffer = b""
latest = None                    # most recent complete message line (bytes)


def callback(data):
    # Runs in IRQ context: only buffer and split. No motor, sensor or heavy work here.
    global buffer, latest
    buffer += data
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        latest = line


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def run():
    global latest
    from BLE_CEEO import Yell

    sL, sR = make_sensors()
    load_training()
    benchmark_knn()

    p = None
    try:
        p = Yell(NAME, interval_us=30000, verbose=False)
        p.callback = callback

        while True:
            stop()
            print("Waiting for the controller...")
            np_set(0, 0, 40)                     # blue = advertising
            if not p.connect_up():
                continue
            print("Connected")
            np_clear()

            last_msg = time.ticks_ms()
            ignore_until = time.ticks_ms()
            last_color = None
            hits = 0
            armed = True

            while p.is_connected:
                t0 = time.ticks_ms()

                # --- joystick message -> motors ---
                if latest is not None:
                    line, latest = latest, None
                    try:
                        parts = line.decode().split(",")
                        if parts[0] == "J" and len(parts) == 4:
                            x, y, boost = int(parts[1]), int(parts[2]), int(parts[3])
                            gain = BOOST_GAIN if boost else NORMAL_GAIN
                            l = clamp((y + x) / 100.0, -1.0, 1.0) * gain
                            r = clamp((y - x) / 100.0, -1.0, 1.0) * gain
                            drive(l, r)
                            last_msg = time.ticks_ms()
                    except (ValueError, UnicodeError):
                        pass                      # ignore a garbled message

                # --- failsafe ---
                if time.ticks_diff(time.ticks_ms(), last_msg) > FAILSAFE_MS:
                    stop()

                # --- finish line (red tape) via KNN ---
                rl, rr = sL.read(), sR.read()
                fl = refl(rl[3], CAL["L"][0], CAL["L"][1])
                fr = refl(rr[3], CAL["R"][0], CAL["R"][1])
                cL = knn_classify(rl[:3], fl, "L")[0]
                cR = knn_classify(rr[:3], fr, "R")[0]
                red = (cL == "RED") or (cR == "RED")

                if red and last_color == "RED":
                    hits += 1
                else:
                    hits = 1 if red else 0
                last_color = "RED" if red else None
                if not red:
                    armed = True

                now = time.ticks_ms()
                if red and armed and hits >= CONFIRM_RED and time.ticks_diff(now, ignore_until) > 0:
                    stop()
                    p.send("F\n")                 # tell the controller to celebrate
                    print("Finish line!")
                    maneuver_red()                # car's own flash + beep
                    armed = False
                    hits = 0
                    last_color = None
                    ignore_until = time.ticks_add(time.ticks_ms(), COOLDOWN_MS)
                    last_msg = time.ticks_ms()

                wait_loop(t0)

            print("Lost connection")
    finally:
        stop()
        np_clear()
        if p:
            p.disconnect()


if MODE == "calibrate":
    calibrate_knn()
elif MODE == "evaluate":
    evaluate_knn()
elif MODE == "sensors":
    sensor_view()
elif MODE == "motors":
    motor_test()
elif MODE == "run":
    run()