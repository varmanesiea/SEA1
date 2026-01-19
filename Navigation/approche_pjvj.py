#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import os
import numpy as np
from picamera2 import Picamera2
import multiprocessing
import pigpio
import time
import sys
import termios
import tty

FL, BL, FR, BR = 23, 25, 22, 17
SL, SR = 24, 16

Kp = 0.2
PERIOD = 0.03
SPEED_FORWARD = 140
SPEED_BACKWARD = 90

RESOLUTION = (1056, 810) if multiprocessing.cpu_count() >= 2 else (480, 368)

LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])
MIN_AREA_POSTIT = 2000

TOLERANCE_X_PIXELS = 30
SPEED_TURN = 60
DISTANCE_CIBLE_CM = 24
TOLERANCE_DISTANCE_CM = 4
PUSH_DURATION = 0.80

POSTIT_REAL_WIDTH_CM = 7.5
FOCAL_LENGTH = 673.97

# Constants virage GAUCHE RÉDUIT (120° → 90°)
TICKS_VIRAGE_GAUCHE_GAUCHE = -6   # Réduit de -8 à -6
TICKS_VIRAGE_GAUCHE_DROITE = 6    # Réduit de 8 à 6
PWM_VIRAGE_GAUCHE = 80

# Constants virage DROITE
TICKS_VIRAGE_DROITE_GAUCHE = 10
TICKS_VIRAGE_DROITE_DROITE = -10
PWM_VIRAGE_DROITE = 80

# Constants rotation 360°
TICKS_360_GAUCHE = -24  # Ajusté: 4x le nouveau 90° (-6x4)
TICKS_360_DROITE = 24
PWM_360 = 80

# Constants avance précise 10cm
TICKS_10CM = 200
K_CORR = 15  # Retour valeur modérée

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpio")

count_left = count_right = 0

def cb_left(gpio, level, tick):
    global count_left
    if level != pigpio.TIMEOUT:
        count_left += 1

def cb_right(gpio, level, tick):
    global count_right
    if level != pigpio.TIMEOUT:
        count_right += 1

pi.set_pull_up_down(SL, pigpio.PUD_DOWN)
pi.set_pull_up_down(SR, pigpio.PUD_DOWN)
cbL = pi.callback(SL, pigpio.EITHER_EDGE, cb_left)
cbR = pi.callback(SR, pigpio.EITHER_EDGE, cb_right)

def set_pwm(lf, lb, rf, rb):
    pi.set_PWM_dutycycle(FL, lf)
    pi.set_PWM_dutycycle(BL, lb)
    pi.set_PWM_dutycycle(FR, rf)
    pi.set_PWM_dutycycle(BR, rb)

def stop():
    set_pwm(0, 0, 0, 0)

def avancer_asservi(duree=0.25):
    global count_left, count_right
    pwm_left = pwm_right = SPEED_FORWARD
    t_start = time.time()
    last_time = t_start
    count_left = count_right = 0
    while time.time() - t_start < duree:
        set_pwm(pwm_left, 0, pwm_right, 0)
        time.sleep(PERIOD)
        now = time.time()
        dt = now - last_time
        last_time = now
        cl, cr = count_left, count_right
        count_left = count_right = 0
        if dt == 0:
            continue
        vl = cl / dt
        vr = cr / dt
        error = vr - vl
        correction = Kp * error
        pwm_left = max(0, min(255, int(pwm_left + correction)))
        pwm_right = max(0, min(255, int(pwm_right - correction)))
    stop()

def tourner_gauche(duree=0.12, vitesse=SPEED_TURN):
    t0 = time.time()
    while time.time() - t0 < duree:
        set_pwm(0, vitesse, vitesse, 0)
        time.sleep(0.01)
    stop()

def tourner_droite(duree=0.12, vitesse=SPEED_TURN):
    t0 = time.time()
    while time.time() - t0 < duree:
        set_pwm(vitesse, 0, 0, vitesse)
        time.sleep(0.01)
    stop()

# Virage GAUCHE 90° précis CALIBRÉ
def virage_gauche_precis():
    global count_left, count_right
    print("🔄 PHASE VIRAGE 90° GAUCHE (ticks réduits -6/+6)...")
    count_left = count_right = 0
    while abs(count_left) < abs(TICKS_VIRAGE_GAUCHE_GAUCHE) or count_right < TICKS_VIRAGE_GAUCHE_DROITE:
        set_pwm(0, PWM_VIRAGE_GAUCHE, PWM_VIRAGE_GAUCHE, 0)
        time.sleep(PERIOD)
    stop()
    print(f"✅ Virage GAUCHE OK: G={count_left}, D={count_right}")

# Virage DROITE 90° précis
def virage_droite_precis():
    global count_left, count_right
    print("🔄 PHASE VIRAGE 90° DROITE...")
    count_left = count_right = 0
    while count_left < TICKS_VIRAGE_DROITE_GAUCHE or abs(count_right) < abs(TICKS_VIRAGE_DROITE_DROITE):
        set_pwm(PWM_VIRAGE_DROITE, 0, 0, PWM_VIRAGE_DROITE)
        time.sleep(PERIOD)
    stop()
    print(f"✅ Virage DROITE OK: G={count_left}, D={count_right}")

# Rotation 360° sur place
def rotation_360():
    global count_left, count_right
    print("🎉 ROTATION 360° FINALE...")
    count_left = count_right = 0
    while abs(count_left) < abs(TICKS_360_GAUCHE) or count_right < TICKS_360_DROITE:
        set_pwm(0, PWM_360, PWM_360, 0)
        time.sleep(PERIOD)
    stop()
    print(f"✅ Rotation 360° OK: G={count_left}, D={count_right}")

# Avance 10cm précise
def avancer_10cm_precis():
    global count_left, count_right
    print(f"➡️ AVANCE 10cm (cible {TICKS_10CM} ticks, K_CORR={K_CORR})...")
    pwm_left = 152
    pwm_right = 160
    count_left = count_right = 0
    last_count_left = last_count_right = 0
    
    while (count_left + count_right) // 2 < TICKS_10CM:
        set_pwm(pwm_left, 0, pwm_right, 0)
        time.sleep(PERIOD)
        
        dl = count_left - last_count_left
        dr = count_right - last_count_right
        last_count_left = count_left
        last_count_right = count_right
        
        diff = dl - dr
        
        if abs(diff) > 1:
            corr = K_CORR * diff
            pwm_left = 152 - corr
            pwm_right = 160 + corr
        else:
            pwm_left = 152
            pwm_right = 160
        
        pwm_left = max(50, min(255, pwm_left))
        pwm_right = max(50, min(255, pwm_right))
    
    stop()
    diff_final = count_left - count_right
    print(f"✅ 10cm OK: G={count_left}, D={count_right} (diff={diff_final})")

def detect_postit_yellow(hsv):
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > MIN_AREA_POSTIT and area > max_area:
            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w//2, y + h//2
            distance_proxy = POSTIT_REAL_WIDTH_CM * FOCAL_LENGTH / w if w > 0 else 999
            best = (cx, cy, w, h, area, int(distance_proxy))
            max_area = area
    return best

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": RESOLUTION},
    controls={"FrameRate": 60},
    buffer_count=1
))
picam2.start()
time.sleep(1)

print("=== JAUNE1(pousse) → G90°(-6/+6) → 10cm → JAUNE2(centre) → D90° → JAUNE3(rotation360°) ===")
print("z: mission complète, q: quitter")
print("BALAYAGE: DROITE | VIRAGE G: ticks -6/+6 (calibré pour 90° exact)")

try:
    while True:
        ch = getch()
        if ch == 'q':
            break
        if ch == 'z':
            os.system('clear')
            print("🎯 MISSION: JAUNE1(pousse) → G90°(-6/+6) → 10cm → JAUNE2 → D90° → JAUNE3(360°)")
            width_image = RESOLUTION[0]
            center_x = width_image // 2
            mission_complete = False
            phase = "JAUNE1"

            while not mission_complete:
                image = picam2.capture_array()
                if len(image.shape) == 3 and image.shape[2] == 4:
                    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
                image = cv2.rotate(image, cv2.ROTATE_180)
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                postit = detect_postit_yellow(hsv)
                color_name = "JAUNE"

                os.system('clear')
                print(f"PHASE: {phase} ({color_name}, tol=30px)")
                
                if postit is None:
                    print("❌ PERDU → balayage DROITE")
                    tourner_droite(0.15, SPEED_TURN)
                else:
                    bx = postit[0]
                    dist = postit[5]
                    dx = bx - center_x
                    print(f"{color_name}: X={bx}px (dx={dx}/30), dist={dist}cm")

                    if abs(dx) > TOLERANCE_X_PIXELS:
                        if dx < 0:
                            print(f"→ GAUCHE (dx={dx} < 0)")
                            tourner_gauche(0.08, SPEED_TURN)
                        else:
                            print(f"→ DROITE (dx={dx} > 0)")
                            tourner_droite(0.08, SPEED_TURN)
                    else:
                        print(f"✅ {color_name} CENTRÉ dx={dx}")
                        
                        if phase == "JAUNE1":
                            if dist > DISTANCE_CIBLE_CM + TOLERANCE_DISTANCE_CM:
                                print(f"→ AVANCER {dist}>{DISTANCE_CIBLE_CM}cm")
                                avancer_asservi(0.25)
                            elif dist < DISTANCE_CIBLE_CM - TOLERANCE_DISTANCE_CM:
                                print("→ POUSSE FINALE")
                                avancer_asservi(PUSH_DURATION)
                            else:
                                print("✅ JAUNE1 PARFAIT → VIRAGE G(-6/+6) + 10cm")
                                virage_gauche_precis()
                                avancer_10cm_precis()
                                phase = "JAUNE2"
                        
                        elif phase == "JAUNE2":
                            print("✅ JAUNE2 CENTRÉ → VIRAGE DROITE direct")
                            virage_droite_precis()
                            phase = "JAUNE3"
                        
                        elif phase == "JAUNE3":
                            print("🚀 JAUNE3 CENTRÉ → ROTATION 360° FINALE")
                            rotation_360()
                            print("\n" + "="*50)
                            print("🎉 PROGRAMME TERMINÉ 🎉")
                            print("="*50)
                            mission_complete = True
                            stop()

                print("C: stop")
                time.sleep(0.05)

except KeyboardInterrupt:
    print("\nArrêt...")

stop()
cbL.cancel()
cbR.cancel()
picam2.stop()
pi.stop()
print("Robot arrêté proprement.")