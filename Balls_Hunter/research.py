#!/usr/bin/python3

import cv2
import os
from picamera2 import Picamera2
import multiprocessing
import numpy as np
import pigpio
import time

# =======================
# Config caméra
# =======================
RESOLUTION = (480, 368)
if multiprocessing.cpu_count() >= 2:
    RESOLUTION = (1056, 810)

# =======================
# Config moteurs + encodeurs
# =======================
pin_backward_left = 25
pin_backward_right = 17
pin_forward_left = 23
pin_forward_right = 22

SL = 24   # encodeur gauche
SR = 16   # encodeur droit

pi = pigpio.pi()

count_left = 0
count_right = 0

def cb_left(gpio, level, tick):
    global count_left
    count_left += 1

def cb_right(gpio, level, tick):
    global count_right
    count_right += 1

pi.set_pull_up_down(SL, pigpio.PUD_DOWN)
pi.set_pull_up_down(SR, pigpio.PUD_DOWN)
cbL = pi.callback(SL, pigpio.EITHER_EDGE, cb_left)
cbR = pi.callback(SR, pigpio.EITHER_EDGE, cb_right)

def set_pwm(left_forward, left_backward, right_forward, right_backward):
    pi.set_PWM_dutycycle(pin_forward_left,  left_forward)
    pi.set_PWM_dutycycle(pin_backward_left, left_backward)
    pi.set_PWM_dutycycle(pin_forward_right, right_forward)
    pi.set_PWM_dutycycle(pin_backward_right, right_backward)

def stop():
    set_pwm(0, 0, 0, 0)

# =======================
# Rotations (sans avance)
# =======================
def tourner_gauche_continu(duree_s=0.12, vitesse=120):
    t0 = time.time()
    while time.time() - t0 < duree_s:
        # roue gauche arrière, roue droite avant
        set_pwm(0, vitesse, vitesse, 0)
        time.sleep(0.01)
    stop()

def tourner_droite_continu(duree_s=0.12, vitesse=120):
    t0 = time.time()
    while time.time() - t0 < duree_s:
        # roue gauche avant, roue droite arrière
        set_pwm(vitesse, 0, 0, vitesse)
        time.sleep(0.01)
    stop()

# =======================
# Détection de balle verte
# =======================
BALL_DIAMETER_MM = 65
FOCAL_LENGTH = 673.97   # focale issue de ta calibration [web:23]

# Plage HSV pour le VERT (à ajuster si besoin) [web:22][web:26]
LOWER_GREEN = np.array([40, 70, 70])
UPPER_GREEN = np.array([80, 255, 255])

MIN_AREA = 500
MAX_AREA = 150000

def calculate_distance(diameter_pixels):
    if diameter_pixels == 0:
        return 0
    distance_mm = (BALL_DIAMETER_MM * FOCAL_LENGTH) / diameter_pixels
    return int(distance_mm / 10)

def detect_green_ball(hsv_image):
    mask = cv2.inRange(hsv_image, LOWER_GREEN, UPPER_GREEN)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_ball = None
    max_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if MIN_AREA < area < MAX_AREA:
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity > 0.7 and area > max_area:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        diameter = int(2 * np.sqrt(area / np.pi))
                        distance = calculate_distance(diameter)
                        best_ball = {
                            'x': cx,
                            'y': cy,
                            'diameter': diameter,
                            'distance': distance
                        }
                        max_area = area
    return best_ball

# =======================
# Paramètres de centrage
# =======================
TOLERANCE_X_PIXELS = 30     # tolérance horizontale autour du centre image
SPEED_TURN = 70

# =======================
# Caméra
# =======================
try:
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"format": 'XRGB8888', "size": RESOLUTION},
        controls={'FrameRate': 60, 'FrameDurationLimits': (100, 50000)},
        buffer_count=1))
    picam2.start()
    time.sleep(1)
except Exception as e:
    print("Erreur caméra:", str(e))
    cbL.cancel()
    cbR.cancel()
    pi.stop()
    raise

print("=" * 50)
print("MISSION CHASSEUR DE BALLES – ÉTAPE 1")
print("=" * 50)
print("Recherche d’une balle VERTE, centrage puis arrêt.\n")

try:
    width_image = RESOLUTION[0]
    center_x = width_image // 2

    while True:
        image = picam2.capture_array()

        # conversion BGRA->BGR si besoin
        if len(image.shape) == 3 and image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        image = cv2.rotate(image, cv2.ROTATE_180)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        green_ball = detect_green_ball(hsv)

        os.system('clear')
        print("=" * 50)
        print("CHASSEUR DE BALLES – CENTRAGE")
        print("=" * 50)

        if green_ball is None:
            print("\n✗ Aucune balle verte détectée")
            print("→ Action : recherche, rotation lente à gauche")
            tourner_gauche_continu(duree_s=0.15, vitesse=SPEED_TURN)

        else:
            bx = green_ball['x']
            dist = green_ball['distance']
            dx = bx - center_x

            print(f"\n✓ Balle verte détectée")
            print(f"  Position image X : {bx} px (centre image : {center_x} px)")
            print(f"  Décalage horizontal dx : {dx} px")
            print(f"  Distance estimée : {dist} cm")

            if abs(dx) > TOLERANCE_X_PIXELS:
                if dx < 0:
                    print("\n→ Action : TOURNER GAUCHE pour centrer la balle")
                    tourner_gauche_continu(duree_s=0.08, vitesse=SPEED_TURN)
                else:
                    print("\n→ Action : TOURNER DROITE pour centrer la balle")
                    tourner_droite_continu(duree_s=0.08, vitesse=SPEED_TURN)
            else:
                print("\n→ Balle centrée dans la tolérance horizontale")
                print("→ Action : ARRÊT – MISSION TERMINÉE ✅")
                stop()
                break

        print("\nCtrl+C pour arrêter manuellement")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nArrêt manuel du programme...")

stop()
cbL.cancel()
cbR.cancel()
picam2.stop()
pi.stop()
print("Programme terminé.")