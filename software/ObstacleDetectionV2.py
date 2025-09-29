import os
import time
import cv2
import numpy as np
import keyboard

# RealSense
import pyrealsense2 as rs

# ----- konfiguracija -----
DEVICE_IS_RPI = False
TARGET_FPS = 6               # 6, 15, 30, ...
PRESET = 1                   # 1=848x480, 2=640x360, 3=480x270
THR_NEAR = 1.0
THR_MID = 1.6
X_STEP = 32
Y_STEP = 24

# GPIO pini (BOARD številčenje – ker so 29/31/...)
PIN_LL = 29
PIN_LE = 31
PIN_NA = 33
PIN_DE = 35
PIN_DD = 37
PWM_FREQ = 100

# ----- stanje izhodov -----
LLstate = LEstate = NAstate = DEstate = DDstate = 1  # 1=brez, 2=srednje, 3=blizu

print("\n\nVISUS software rev2 2021 - See With Me V3")
print("Software by Nik Stanojevic & Val Vidmar | Press 'x' to exit\n")

# ----- opcijski RPi GPIO -----
try:
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)  # pomembno: BOARD, ker so pini 29/31/...
    for p in (PIN_LL, PIN_LE, PIN_NA, PIN_DE, PIN_DD):
        GPIO.setup(p, GPIO.OUT)

    pwmLL = GPIO.PWM(PIN_LL, PWM_FREQ)
    pwmLE = GPIO.PWM(PIN_LE, PWM_FREQ)
    pwmNA = GPIO.PWM(PIN_NA, PWM_FREQ)
    pwmDE = GPIO.PWM(PIN_DE, PWM_FREQ)
    pwmDD = GPIO.PWM(PIN_DD, PWM_FREQ)
    for pwm in (pwmLL, pwmLE, pwmNA, pwmDE, pwmDD):
        pwm.start(0)

    DEVICE_IS_RPI = True
    os.system('cls' if os.name == 'nt' else 'clear')
    print("Raspberry Pi mode initialized...\n")
except Exception as e:
    print("Note: GPIO not available (probably not running on Raspberry Pi).")
    DEVICE_IS_RPI = False

def set_pwm(pwm, duty):
    if DEVICE_IS_RPI:
        pwm.ChangeDutyCycle(max(0, min(100, duty)))

def all_pwm_zero():
    if DEVICE_IS_RPI:
        for pwm in (pwmLL, pwmLE, pwmNA, pwmDE, pwmDD):
            pwm.ChangeDutyCycle(0)

# ----- RealSense pipeline -----
def start_camera():
    if PRESET == 1:
        w, h = 848, 480
    elif PRESET == 2:
        w, h = 640, 360
    else:
        w, h = 480, 270

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, TARGET_FPS)
    pipeline.start(cfg)
    return pipeline, w, h

pipeline, width, height = None, 0, 0
camera_ok = False
while not camera_ok:
    if keyboard.is_pressed('x'):
        break
    try:
        pipeline, width, height = start_camera()
        camera_ok = True
    except Exception:
        os.system('cls' if os.name == 'nt' else 'clear')
        print("Camera not connected yet… searching.")
        time.sleep(0.5)

# ----- glavni loop -----
Zones = None
OneWidth = 0
frameNum = 0
totalFPS = 0.0

try:
    OneWidth = int(width / 5)
    Zones = [1, OneWidth, OneWidth*2, OneWidth*3, OneWidth*4, width]

    while True:
        t0 = time.time()
        if keyboard.is_pressed('x'):
            break

        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth:
            continue

        depth_img = np.asanyarray(depth.get_data())
        vis = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_INFERNO)

        # per-frame akumulatorji
        MinDist  = [100]*6
        CntNear  = [0]*6
        CntMid   = [0]*6
        shortDistance = [0]*5
        mediumDistance = [0]*5
        longDistance   = [0]*5

        # skeniraj mrežo pik
        x, y = 31, 23
        while y < height and x < width:
            thr_mid = 1.1 if y > 300 else THR_MID
            # v katerem pasu je točka?
            o = 1 + min(4, x // OneWidth)

            d = depth.get_distance(x, y)
            if d == 0:
                d = 1e9  # manjkajoči vzorec

            MinDist[o] = min(MinDist[o], d)
            if d <= THR_NEAR:
                CntNear[o] += 1
                cv2.rectangle(vis, (x-2, y-2), (x+2, y+2), (0, 0, 255), 2)
            elif d <= thr_mid:
                CntMid[o]  += 1
                cv2.rectangle(vis, (x-2, y-2), (x+2, y+2), (255, 0, 0), 2)
            else:
                cv2.rectangle(vis, (x-2, y-2), (x+2, y+2), (0, 0, 0), 2)

            x += X_STEP
            if x >= width:
                x = 31
                y += Y_STEP

        # odločanje po conah
        for o in range(1, 6):
            if CntNear[o] > 10:
                # blizu
                shortDistance[o-1] = 1
                cv2.rectangle(vis, (Zones[o-1], 220), (Zones[o], 260), (0, 0, 255), 5)
                duty = 100
                level = 3
            elif CntMid[o] > 10:
                # srednje
                mediumDistance[o-1] = 1
                cv2.rectangle(vis, (Zones[o-1], 220), (Zones[o], 260), (255, 0, 0), 5)
                duty = 50
                level = 2
            else:
                # daleč / nič
                longDistance[o-1] = 1
                cv2.rectangle(vis, (Zones[o-1], 220), (Zones[o], 260), (0, 0, 0), 5)
                duty = 0
                level = 1

            # PWM izhodi
            if DEVICE_IS_RPI:
                if   o == 1: LLstate = level; set_pwm(pwmLL, duty)
                elif o == 2: LEstate = level; set_pwm(pwmLE, duty)
                elif o == 3: NAstate = level; set_pwm(pwmNA, duty)
                elif o == 4: DEstate = level; set_pwm(pwmDE, duty)
                elif o == 5: DDstate = level; set_pwm(pwmDD, duty)

        cv2.imshow('RealSense1', vis)
        cv2.waitKey(1)

        fps = 1.0 / max(1e-6, time.time() - t0)
        frameNum += 1
        totalFPS += fps
        if TARGET_FPS > (fps + 1):
            print("Low FPS warning!!!")

        os.system('cls' if os.name == 'nt' else 'clear')
        print("------------------------")
        print("Long:  ", longDistance)
        print("Mid:   ", mediumDistance)
        print("Near:  ", shortDistance)
        print("RP out:", LLstate, LEstate, NAstate, DEstate, DDstate)
        print("FPS:   ", round(fps, 2))

except KeyboardInterrupt:
    pass
finally:
    # cleanup
    try:
        if pipeline: pipeline.stop()
    except Exception:
        pass
    cv2.destroyAllWindows()
    if DEVICE_IS_RPI:
        try:
            all_pwm_zero()
            for pwm in (pwmLL, pwmLE, pwmNA, pwmDE, pwmDD):
                pwm.stop()
            GPIO.cleanup()
        except Exception:
            pass

    if frameNum:
        print("Average FPS:", round(totalFPS/frameNum, 2))
    print("Sequence closed successfully.")
