#!/usr/bin/python3
# -*- coding: utf-8 -*-

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
# Config moteurs
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
# Rotations sur place
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
# Détection post-it jaune
# =======================

# Plage HSV pour le JAUNE (OpenCV: H 0-179, S 0-255, V 0-255)
# Point de départ classique, à affiner avec ton éclairage. [web:6][web:4]
LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

MIN_AREA_POSTIT = 2000   # min area en px² pour ignorer le bruit

def detect_yellow_postit(hsv_image):
    # masque des pixels jaunes
    mask = cv2.inRange(hsv_image, LOWER_YELLOW, UPPER_YELLOW)

    # nettoyage du masque
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    best = None
    max_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > MIN_AREA_POSTIT:
            x, y, w, h = cv2.boundingRect(contour)
            if area > max_area:
                cx = x + w // 2
                cy = y + h // 2
                best = {
                    "x": cx,
                    "y": cy,
                    "w": w,
                    "h": h,
                    "area": area
                }
                max_area = area

    return best

# =======================
# Paramètres de centrage
# =======================
TOLERANCE_X_PIXELS = 30
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
print("NAVIGATION PAR MARQUEURS – POST-IT JAUNE")
print("=" * 50)
print("Recherche d’un post-it JAUNE, centrage puis arrêt.\n")

try:
    width_image = RESOLUTION[0]
    center_x = width_image // 2

    while True:
        image = picam2.capture_array()

        # conversion BGRA -> BGR si besoin
        if len(image.shape) == 3 and image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        # rotation suivant ton montage
        image = cv2.rotate(image, cv2.ROTATE_180)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        postit = detect_yellow_postit(hsv)

        os.system('clear')
        print("=" * 50)
        print("RECHERCHE POST-IT JAUNE – CENTRAGE")
        print("=" * 50)

        if postit is None:
            print("\n✗ Aucun post-it jaune détecté")
            print("→ Action : rotation lente à gauche pour chercher")
            tourner_gauche_continu(duree_s=0.15, vitesse=SPEED_TURN)

        else:
            bx = postit["x"]
            dx = bx - center_x

            print(f"\n✓ Post-it jaune détecté")
            print(f"  Centre X : {bx} px (centre image : {center_x} px)")
            print(f"  Décalage dx : {dx} px")

            if abs(dx) > TOLERANCE_X_PIXELS:
                if dx < 0:
                    print("\n→ TOURNER GAUCHE pour centrer le post-it")
                    tourner_gauche_continu(duree_s=0.08, vitesse=SPEED_TURN)
                else:
                    print("\n→ TOURNER DROITE pour centrer le post-it")
                    tourner_droite_continu(duree_s=0.08, vitesse=SPEED_TURN)
            else:
                print("\n→ Post-it centré dans la tolérance")
                print("→ Action : ARRÊT – MISSION TERMINÉE ✅")
                stop()
                break

        print("\nCtrl+C pour arrêter manuellement")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nArrêt manuel du programme...")

# =======================
# Nettoyage
# =======================
stop()
cbL.cancel()
cbR.cancel()
picam2.stop()
pi.stop()
print("Programme terminé.")