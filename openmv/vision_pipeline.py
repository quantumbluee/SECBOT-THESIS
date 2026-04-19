# Final_vision - By: urvih - Sat Apr 18 2026

import sensor
import time
from pyb import UART

uart = UART(3,115200)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

LINE_ROI = (0,60,160,60)
OBS_ROI = (0,30,160,90)

LINE_THRESH = [(0,60)]
OBS_THRESH = [(0,30)]

LINE_PIXELS = 200
LINE_AREA = 200

OBS_PIXELS = 600
OBS_AREA = 600

while True:
    clock.tick()
    img = sensor.snapshot()

    line_found = 0
    line_error = 0
    obstacle_flag = 0
    confidence = 0

    #-------------Line detection---------------
    line_blobs = img.find_blobs(
    LINE_THRESH,
    roi = LINE_ROI,
    pixels_threshold = LINE_PIXELS,
    area_threshold=LINE_AREA,
    merge=True
    )

    img.draw_rectangle(LINE_ROI)

    if line_blobs:
        largest_line = max(line_blobs, key=lambda b: b.pixels())
        line_found = 1
        center_x = img.width() //2
        line_error = largest_line.cx() - center_x

        img.draw_rectangle(largest_line.rect())
        img.draw_cross(largest_line.cx(), largest_line.cy())

        #crude confidence from blob size
        confidence = min(100, largest_line.pixels() // 10)

    #-------------obstacle detection-----------
    obs_blobs = img.find_blobs(
    OBS_THRESH,
    roi = OBS_ROI,
    pixels_threshold=OBS_PIXELS,
    area_threshold=OBS_AREA,
    merge = True
    )

    img.draw_rectangle(OBS_ROI)

    if obs_blobs:
        largest_obs = max(obs_blobs, key=lambda b:b.pixels())
        obstacle_flag = 1
        img.draw_rectangle(largest_obs.rect())
        img.draw_cross(largest_obs.cx(), largest_obs.cy())

        if confidence ==0:
            confidence = min(100, largest_obs.pixels() // 10)

    msg = "VISION,{},{},{},{}\n".format(
    line_found,
    line_error,
    obstacle_flag,
    confidence
    )

    uart.write(msg)
    print(msg.strip(), "FPS:", clock.fps())
    time.sleep_ms(100)
