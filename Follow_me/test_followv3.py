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

# paramètres asservissement avance
Kp = 0.2
PERIOD = 0.05   # 50 ms

def set_pwm(left_forward, left_backward, right_forward, right_backward):
    pi.set_PWM_dutycycle(pin_forward_left,  left_forward)
    pi.set_PWM_dutycycle(pin_backward_left, left_backward)
    pi.set_PWM_dutycycle(pin_forward_right, right_forward)
    pi.set_PWM_dutycycle(pin_backward_right, right_backward)

def stop():
    set_pwm(0, 0, 0, 0)

# =======================
#  Rotations continues
# =======================

def tourner_gauche_continu(duree_s=0.25, vitesse=120):
    t0 = time.time()
    while time.time() - t0 < duree_s:
        set_pwm(0, vitesse, vitesse, 0)
        time.sleep(0.01)

def tourner_droite_continu(duree_s=0.25, vitesse=120):
    t0 = time.time()
    while time.time() - t0 < duree_s:
        set_pwm(vitesse, 0, 0, vitesse)
        time.sleep(0.01)

# =======================
#  Suivi ArUco
# =======================
TOLERANCE_X = 7
SPEED_TURN = 50      # un peu plus lent pour le scan
DISTANCE_CIBLE = 15
TOLERANCE_Z = 5
SPEED_FORWARD = 70

# temps de dernière détection
last_seen_time = 0.0

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

def suivre_tag_avancer(duree_s=0.3):
    global count_left, count_right

    pwm_left = SPEED_FORWARD
    pwm_right = SPEED_FORWARD

    t_start = time.time()
    last_time_local = t_start
    count_left = 0
    count_right = 0

    while time.time() - t_start < duree_s:
        set_pwm(pwm_left, 0, pwm_right, 0)
        time.sleep(PERIOD)

        now = time.time()
        dt = now - last_time_local
        last_time_local = now

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
    last_seen_time = time.time()

    while True:
        image = picam2.capture_array()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.rotate(gray_image, cv2.ROTATE_180)

        markers = extract_markers(gray_image)

        os.system('clear')
        print("=" * 50)
        print("FOLLOW-ME : ORIENTATION + SUIVI")
        print("=" * 50)

        now = time.time()

        if markers:
            target = markers[0]
            x = target["x"]
            z = target["z"]

            # on vient de voir le tag → reset du timer
            last_seen_time = now

            print(f"\n✓ Tag ID {target['id']} détecté")
            print(f"  Position X: {x:4d} cm (horizontal)")
            print(f"  Distance Z: {z:4d} cm")
            print(f"  Distance cible: {DISTANCE_CIBLE} cm (±{TOLERANCE_Z} cm)")

            if abs(x) > TOLERANCE_X:
                if x < -TOLERANCE_X:
                    print(f"\n→ Action: TOURNER GAUCHE")
                    tourner_gauche_continu(duree_s=0.10, vitesse=SPEED_TURN)
                elif x > TOLERANCE_X:
                    print(f"\n→ Action: TOURNER DROITE")
                    tourner_droite_continu(duree_s=0.10, vitesse=SPEED_TURN)

            elif z > DISTANCE_CIBLE + TOLERANCE_Z:
                print(f"\n→ Action: AVANCER (asservi fluide)")
                suivre_tag_avancer(duree_s=0.3)

            elif z < DISTANCE_CIBLE - TOLERANCE_Z:
                print(f"\n→ Action: TROP PROCHE ✓")
                stop()

            else:
                print(f"\n→ Action: POSITION PARFAITE ✓")
                stop()

        else:
            print("\n✗ Aucun tag détecté")
            stop()

            # marge d'erreur : on laisse 1 s après la dernière vue
            if now - last_seen_time < 1.0:
                print("→ Attente marge d'erreur (1 s depuis la dernière détection)")
            else:
                print("→ Scan local lent : 1 coup droite, 2 coups gauche")

                # paramètres du petit scan (ralenti)
                d_short = 0.07    # durée d'un à‑coup plus longue
                pause   = 0.06
                v_scan  = SPEED_TURN

                # 1) petit à‑coup vers la DROITE
                tourner_droite_continu(duree_s=d_short, vitesse=v_scan)
                time.sleep(pause)

                image = picam2.capture_array()
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                gray = cv2.rotate(gray, cv2.ROTATE_180)
                markers_scan = extract_markers(gray)

                if not markers_scan:
                    # 2) premier coup à GAUCHE (revient vers centre)
                    tourner_gauche_continu(duree_s=d_short, vitesse=v_scan)
                    time.sleep(pause)

                    image = picam2.capture_array()
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    gray = cv2.rotate(gray, cv2.ROTATE_180)
                    markers_scan = extract_markers(gray)

                    if not markers_scan:
                        # 3) deuxième coup à GAUCHE (va un peu plus à gauche)
                        tourner_gauche_continu(duree_s=d_short, vitesse=v_scan)
                        time.sleep(pause)

                        image = picam2.capture_array()
                        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                        gray = cv2.rotate(gray, cv2.ROTATE_180)
                        markers_scan = extract_markers(gray)

                        if not markers_scan:
                            print("→ Toujours rien : balayage complet autorisé")
                            tourner_droite_continu(duree_s=0.6, vitesse=v_scan)

                stop()

        print("\n" + "=" * 50)
        print("Ctrl+C pour arrêter")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nArrêt du programme...")

stop()
cbL.cancel()
cbR.cancel()
picam2.stop()
pi.stop()
print("Programme terminé")