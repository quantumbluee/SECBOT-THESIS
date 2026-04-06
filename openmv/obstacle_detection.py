# ObstacleDetection - By: urvih - Sun Apr 5 2026

import sensor
import time
import image
from pyb import UART

uart = UART(3, 115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whiteball(False)

THRESHOLD = [(0,60)] #DARK OBSTACLE
MIN_PIXELS = 300
MIN_AREA = 300

while True:
    img = sensor.snapshot()
    blobs = img.find_blobs(THRESHOLD, pixels_threshold=MIN_PIXELS, area_threshold=MIN_AREA, merge=True)

    obstacle_flag = 0
    zone = "N"
    score = 0

    if blobs:
        largest = max(blobs, key=lambda b: b.pixels())
        obstacle_flag = 1
        score = largest.pixels()
        cx = largest.cx()

        if cx < img.width()//3:
            zone = "L"
        elif cx < 2*img.width()//3:
            zone="C"
        else:
            zone="R"

        img.draw_rectangle(largest.rect())
        img.draw_cross(largest.cx(), largest.cy())

        msg = "OBS,{},{}\n".format(obstacle_flag, zone, score)
        uart.write(msg)
        print(msg)
        time.sleep_ms(100)
