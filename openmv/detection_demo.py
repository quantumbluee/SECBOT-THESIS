# Untitled - By: urvih - Wed Nov 26 2025

import sensor
import time
from pyb import UART, LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)

uart = UART(3, 115200, bits=8, parity=None, stop=1)
led = LED(1)

i = 0
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    msg = "OVM test %d\r\n" % i
    uart.write(msg)
    print("SENTL", msg.strip())
    led.toggle()

    i += 1
    time.sleep_ms(500)
