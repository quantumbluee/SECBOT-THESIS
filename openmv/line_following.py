# Line Following openmv - By: urvih - Wed Mar 25 2026

import sensor
import time
import image
from pyb import UART

uart = UART(3,115200)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA) #160x120
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whiteball(False)

ROI = (0, 60, 160, 60) #bottom half of the image


while True:
    img = sensor.snapshot()

    #threshold for dark line on bright floor
    #tuning this later
    blobs = img.find_blobs([(0,60)], roi=ROI,pixels_threshold=200, area_threshold=200, merge=True)

    if blobs:
        largest = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(ROI)
        img.draw_rectangle(largest.rect())
        img.draw_cross(largest.cx(), largest.cy())

        center_x = img.width() // 2
        error = largest.cx() - center_x
        angle = 0

        msg = "LINE,1,{},{}\n".format(error,angle)
    else:
        img.draw_rectangle(ROI)
        msg = "LINE,0,0,0\n"

    uart.write(msg)
    print(msg)
    time.sleep_ms(100)


