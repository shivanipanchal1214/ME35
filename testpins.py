from machine import Pin

for n in (2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33):
    try:
        p = Pin(n, Pin.IN, Pin.PULL_UP)
        print(n, p.value())
    except Exception as e:
        print(n, "error:", e)
    