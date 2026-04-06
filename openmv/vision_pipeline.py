import sensor
import image
import time
from pyb import UART

# UART to STM32
uart = UART(3, 115200)

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # 160x120
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# ---------- Line-follow settings ----------
LINE_ROI = (0, 60, 160, 60)          # bottom half
LINE_THRESH = [(0, 60)]              # dark line on bright floor
LINE_PIXELS_MIN = 200
LINE_AREA_MIN = 200

# ---------- Obstacle settings ----------
OBS_THRESH = [(0, 60)]               # dark obstacle
OBS_PIXELS_MIN = 300
OBS_AREA_MIN = 300
OBS_INTERVAL_MS = 200                # obstacle detection at ~5 Hz

last_obs_ms = time.ticks_ms()

while True:
    img = sensor.snapshot()

    # =========================
    # 1) LINE DETECTION (every frame)
    # =========================
    line_valid = 0
    line_error = 0
    line_angle = 0

    line_blobs = img.find_blobs(
        LINE_THRESH,
        roi=LINE_ROI,
        pixels_threshold=LINE_PIXELS_MIN,
        area_threshold=LINE_AREA_MIN,
        merge=True
    )

    img.draw_rectangle(LINE_ROI)

    if line_blobs:
        largest_line = max(line_blobs, key=lambda b: b.pixels())
        line_valid = 1

        img.draw_rectangle(largest_line.rect())
        img.draw_cross(largest_line.cx(), largest_line.cy())

        center_x = img.width() // 2
        line_error = largest_line.cx() - center_x
        line_angle = 0   # placeholder for now

    line_msg = "LINE,{},{},{}\n".format(line_valid, line_error, line_angle)
    uart.write(line_msg)
    print(line_msg)

    # =========================
    # 2) OBSTACLE DETECTION (slower rate)
    # =========================
    now = time.ticks_ms()
    if time.ticks_diff(now, last_obs_ms) >= OBS_INTERVAL_MS:
        obstacle_flag = 0
        zone = "N"
        score = 0

        obs_blobs = img.find_blobs(
            OBS_THRESH,
            pixels_threshold=OBS_PIXELS_MIN,
            area_threshold=OBS_AREA_MIN,
            merge=True
        )

        if obs_blobs:
            largest_obs = max(obs_blobs, key=lambda b: b.pixels())
            obstacle_flag = 1
            score = largest_obs.pixels()
            cx = largest_obs.cx()

            if cx < img.width() // 3:
                zone = "L"
            elif cx < 2 * img.width() // 3:
                zone = "C"
            else:
                zone = "R"

            img.draw_rectangle(largest_obs.rect())
            img.draw_cross(largest_obs.cx(), largest_obs.cy())

        obs_msg = "OBS,{},{},{}\n".format(obstacle_flag, zone, score)
        uart.write(obs_msg)
        print(obs_msg)

        last_obs_ms = now

    time.sleep_ms(50)   # helps keep loop stable