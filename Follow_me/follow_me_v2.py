#!/usr/bin/python3

import cv2
import os
from picamera2 import Picamera2
import multiprocessing
import numpy as np
import pigpio
import time

# Configuration ArUco
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

RESOLUTION = (480, 368)
if multiprocessing.cpu_count() >= 2:
    RESOLUTION = (1056, 810)

# Configuration moteurs
pin_backward_left = 25
pin_backward_right = 17
pin_forward_left = 23
pin_forward_right = 22
heading_bias = -12
forward_correction = 30  # Correction pour avancer droit

# Initialisation GPIO
pi = pigpio.pi()

TOLERANCE_X = 5       
SPEED_TURN = 50       
DISTANCE_CIBLE = 15   
TOLERANCE_Z = 5       
SPEED_FORWARD = 50    

def avancer(vitesse=100):
    pi.set_PWM_dutycycle(pin_forward_left, vitesse+heading_bias+forward_correction)
    pi.set_PWM_dutycycle(pin_forward_right, vitesse-heading_bias)
    pi.set_PWM_dutycycle(pin_backward_left, 0)
    pi.set_PWM_dutycycle(pin_backward_right, 0)

def gauche(vitesse=100):
    pi.set_PWM_dutycycle(pin_forward_left, 0)
    pi.set_PWM_dutycycle(pin_forward_right, vitesse)
    pi.set_PWM_dutycycle(pin_backward_left, vitesse)
    pi.set_PWM_dutycycle(pin_backward_right, 0)

def droite(vitesse=100):
    pi.set_PWM_dutycycle(pin_forward_left, vitesse)
    pi.set_PWM_dutycycle(pin_forward_right, 0)
    pi.set_PWM_dutycycle(pin_backward_left, 0)
    pi.set_PWM_dutycycle(pin_backward_right, vitesse)

def stop():
    pi.set_PWM_dutycycle(pin_forward_left, 0)
    pi.set_PWM_dutycycle(pin_forward_right, 0)
    pi.set_PWM_dutycycle(pin_backward_left, 0)
    pi.set_PWM_dutycycle(pin_backward_right, 0)

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
            ret = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_length, 
                                                       camera_matrix, distortion_coeff)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            tvec = np.squeeze(tvec)
            
            x = int(100 * tvec[0])  # Position horizontale en cm
            y = int(100 * tvec[1])
            z = int(64 * tvec[2])
            if multiprocessing.cpu_count() >= 2:
                z = int(142.2 * tvec[2])
            
            output.append({"x": x, "y": y, "z": z, "id": int(markerID)})
    return output

# Initialisation caméra
try:
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"format": 'XRGB8888', "size": RESOLUTION}, 
        controls={'FrameRate': 60, 'FrameDurationLimits': (100, 50000)}, 
        buffer_count=1))
    picam2.start()
    time.sleep(1)  # Laisser la caméra s'initialiser
except Exception as e:
    print("Erreur caméra:", str(e))
    exit(1)

print("=" * 50)
print("MISSION FOLLOW-ME COMPLETE")
print("=" * 50)
print("Le robot va détecter, s'orienter et suivre le tag")
print("Appuyez sur Ctrl+C pour arrêter\n")

try:
    while True:
        # Capture image
        image = picam2.capture_array()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.rotate(gray_image, cv2.ROTATE_180)
        
        # Détection markers
        markers = extract_markers(gray_image)
        
        # Affichage
        os.system('clear')
        print("=" * 50)
        print("FOLLOW-ME : ORIENTATION + SUIVI")
        print("=" * 50)
        
        if markers:
            # Prendre le premier tag détecté
            target = markers[0]
            x = target["x"]
            z = target["z"]
            
            print(f"\n✓ Tag ID {target['id']} détecté")
            print(f"  Position X: {x:4d} cm (horizontal)")
            print(f"  Distance Z: {z:4d} cm")
            print(f"  Distance cible: {DISTANCE_CIBLE} cm (±{TOLERANCE_Z} cm)")
            
            # Priorité 1 : Orientation (centrage horizontal)
            if abs(x) > TOLERANCE_X:
                if x < -TOLERANCE_X:
                    # Tag à gauche : tourner à gauche
                    print(f"\n→ Action: TOURNER GAUCHE")
                    print(f"  (tag décalé de {abs(x)} cm à gauche)")
                    gauche(SPEED_TURN)
                    
                elif x > TOLERANCE_X:
                    # Tag à droite : tourner à droite
                    print(f"\n→ Action: TOURNER DROITE")
                    print(f"  (tag décalé de {x} cm à droite)")
                    droite(SPEED_TURN)
            
            # Priorité 2 : Suivi en distance (une fois centré)
            elif z > DISTANCE_CIBLE + TOLERANCE_Z:
                # Trop loin : avancer
                print(f"\n→ Action: AVANCER")
                print(f"  (distance actuelle: {z} cm, cible: {DISTANCE_CIBLE} cm)")
                avancer(SPEED_FORWARD)
            
            elif z < DISTANCE_CIBLE - TOLERANCE_Z:
                # Trop proche : on pourrait reculer (optionnel)
                print(f"\n→ Action: TROP PROCHE ✓")
                print(f"  (distance OK: {z} cm)")
                stop()
            
            else:
                # Distance parfaite : arrêter
                print(f"\n→ Action: POSITION PARFAITE ✓")
                print(f"  Centré horizontalement et à bonne distance")
                stop()
        else:
            # Pas de tag : arrêter
            print("\n✗ Aucun tag détecté")
            print("\n→ Action: ARRÊT (recherche)")
            stop()
        
        print("\n" + "=" * 50)
        print("Ctrl+C pour arrêter")
        
        time.sleep(0.1)  # 10 Hz

except KeyboardInterrupt:
    print("\n\nArrêt du programme...")
    stop()
    picam2.stop()
    pi.stop()
    print("Programme terminé")
