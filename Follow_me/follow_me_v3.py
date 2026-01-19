#!/usr/bin/python3

import cv2
import os
from picamera2 import Picamera2
import multiprocessing
import numpy as np
import pigpio
import time

# =======================
#  Config ArUco
# =======================
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

RESOLUTION = (480, 368)
if multiprocessing.cpu_count() >= 2:
    RESOLUTION = (1056, 810)

# =======================
#  Config moteurs + encodeurs
# =======================
pin_backward_left = 25
pin_backward_right = 17
pin_forward_left = 23
pin_forward_right = 22

# plus de correction fixe, tout passe par encodeurs
heading_bias = 0

SL = 24   # encodeur gauche
SR = 16   # encodeur droit

pi = pigpio.pi()

# compteurs encodeurs
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

# paramètres asservissement
Kp = 0.2          # un peu plus doux
PERIOD = 0.1      # 100 ms

def set_pwm(left_forward, left_backward, right_forward, right_backward):
    pi.set_PWM_dutycycle(pin_forward_left,  left_forward)
    pi.set_PWM_dutycycle(pin_backward_left, left_backward)
    pi.set_PWM_dutycycle(pin_forward_right, right_forward)
    pi.set_PWM_dutycycle(pin_backward_right, right_backward)

def stop():
    set_pwm(0, 0, 0, 0)

def gauche(vitesse=100):
    set_pwm(0, vitesse, vitesse, 0)

def droite(vitesse=100):
    set_pwm(vitesse, 0, 0, vitesse)

# =======================
#  Suivi ArUco
# =======================

TOLERANCE_X = 5
SPEED_TURN = 50
DISTANCE_CIBLE = 15
TOLERANCE_Z = 5
SPEED_FORWARD = 140   # PWM moyen (assez fort pour bien rouler)

def extract_markers(image, marker_size_mm=51):
    marker_length = marker_size_mm / 1000.0
    camera_matrix = np.array([[673.9683892, 0., 343.68638231],
                              [0., 676.08466459, 245.31865398],
                              [0., 0., 1.]])
    distortion_coeff = np.array([5.44787247e-02, 1.23043244e-01,
                                 -4.52559581e-04, 5.47011732e-03,
                                 -6.83110234e-01])
    output = []
    (corners, ids, _) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            ret = cv2.aruco.estimatePoseSingleMarkers(
                markerCorner, marker_length,
                camera_matrix, distortion_coeff)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            tvec = np.squeeze(tvec)

            x = int(100 * tvec[0])      # cm
            y = int(100 * tvec[1])
            z = int(64 * tvec[2])
            if multiprocessing.cpu_count() >= 2:
                z = int(142.2 * tvec[2])

            output.append({"x": x, "y": y, "z": z, "id": int(markerID)})
    return output

def suivre_tag_avancer():
    """Avance en continu vers le tag, avec asservissement encodeurs,
       tant que le tag est trop loin et centré."""
    global count_left, count_right

    pwm_left = SPEED_FORWARD
    pwm_right = SPEED_FORWARD

    last_time = time.time()
    count_left = 0
    count_right = 0

    while True:
        # lecture caméra
        image = picam2.capture_array()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.rotate(gray, cv2.ROTATE_180)
        markers = extract_markers(gray)

        if not markers:
            break

        target = markers[0]
        x = target["x"]
        z = target["z"]

        # si plus besoin d'avancer ou plus centré, on laisse la boucle principale décider
        if not (z > DISTANCE_CIBLE + TOLERANCE_Z):
            break
        if abs(x) > TOLERANCE_X:
            break

        # asservissement vitesse pendant PERIOD
        set_pwm(pwm_left, 0, pwm_right, 0)
        time.sleep(PERIOD)

        now = time.time()
        dt = now - last_time
        last_time = now

        cl = count_left
        cr = count_right
        count_left = 0
        count_right = 0

        if dt <= 0:
            continue

        v_left = cl / dt
        v_right = cr / dt
        error = v_right - v_left
        correction = Kp * error

        pwm_left  = max(0, min(255, int(pwm_left  + correction)))
        pwm_right = max(0, min(255, int(pwm_right - correction)))

        # debug optionnel
        # print(f"vL={v_left:.1f} vR={v_right:.1f} err={error:.1f} pwmL={pwm_left} pwmR={pwm_right}")

    stop()

# =======================
#  Caméra
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
print("MISSION FOLLOW-ME COMPLETE")
print("=" * 50)
print("Le robot va détecter, s'orienter et suivre le tag")
print("Appuyez sur Ctrl+C pour arrêter\n")

try:
    while True:
        image = picam2.capture_array()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.rotate(gray_image, cv2.ROTATE_180)

        markers = extract_markers(gray_image)

        os.system('clear')
        print("=" * 50)
        print("FOLLOW-ME : ORIENTATION + SUIVI")
        print("=" * 50)

        if markers:
            target = markers[0]
            x = target["x"]
            z = target["z"]

            print(f"\n✓ Tag ID {target['id']} détecté")
            print(f"  Position X: {x:4d} cm (horizontal)")
            print(f"  Distance Z: {z:4d} cm")
            print(f"  Distance cible: {DISTANCE_CIBLE} cm (±{TOLERANCE_Z} cm)")

            # 1) orientation
            if abs(x) > TOLERANCE_X:
                if x < -TOLERANCE_X:
                    print(f"\n→ Action: TOURNER GAUCHE")
                    print(f"  (tag décalé de {abs(x)} cm à gauche)")
                    gauche(SPEED_TURN)
                elif x > TOLERANCE_X:
                    print(f"\n→ Action: TOURNER DROITE")
                    print(f"  (tag décalé de {x} cm à droite)")
                    droite(SPEED_TURN)

            # 2) distance : avance en continu avec encodeurs
            elif z > DISTANCE_CIBLE + TOLERANCE_Z:
                print(f"\n→ Action: AVANCER (asservi continu)")
                print(f"  (distance actuelle: {z} cm, cible: {DISTANCE_CIBLE} cm)")
                suivre_tag_avancer()

            elif z < DISTANCE_CIBLE - TOLERANCE_Z:
                print(f"\n→ Action: TROP PROCHE ✓")
                print(f"  (distance OK: {z} cm)")
                stop()

            else:
                print(f"\n→ Action: POSITION PARFAITE ✓")
                print(f"  Centré horizontalement et à bonne distance")
                stop()
        else:
            print("\n✗ Aucun tag détecté")
            print("\n→ Action: ARRÊT (recherche)")
            stop()

        print("\n" + "=" * 50)
        print("Ctrl+C pour arrêter")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nArrêt du programme...")

stop()
cbL.cancel()
cbR.cancel()
picam2.stop()
pi.stop()
print("Programme terminé")