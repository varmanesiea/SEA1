#!/usr/bin/python3
# -*- coding: utf-8 -*-

import pigpio
import time
import sys
import termios
import tty

FL = 23
BL = 25
FR = 22
BR = 17
SL = 24
SR = 16

BASE_PWM_LEFT  = 155
BASE_PWM_RIGHT = 150
K_CORR = 8
PERIOD = 0.03
TICKS_APPROCHE = 200

# VIRAGE 90° GAUCHE précis (10 ticks pour ~92°, proche parfait)
TICKS_VIRAGE_DROITE  = 10   # droite AVANT
TICKS_VIRAGE_GAUCHE  = -10  # gauche ARRÈRE
PWM_VIRAGE = 80

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch.lower()

pi = pigpio.pi()
if not pi.connected:
    print("Erreur : pigpio non connecté.")
    sys.exit(1)

pi.set_mode(FL, pigpio.OUTPUT)
pi.set_mode(BL, pigpio.OUTPUT)
pi.set_mode(FR, pigpio.OUTPUT)
pi.set_mode(BR, pigpio.OUTPUT)

pi.set_mode(SL, pigpio.INPUT)
pi.set_mode(SR, pigpio.INPUT)
pi.set_pull_up_down(SL, pigpio.PUD_DOWN)
pi.set_pull_up_down(SR, pigpio.PUD_DOWN)

count_left = 0
count_right = 0

def cb_left(gpio, level, tick):
    global count_left
    if level != pigpio.TIMEOUT:
        count_left += 1

def cb_right(gpio, level, tick):
    global count_right
    if level != pigpio.TIMEOUT:
        count_right += 1

cbL = pi.callback(SL, pigpio.EITHER_EDGE, cb_left)
cbR = pi.callback(SR, pigpio.EITHER_EDGE, cb_right)

def stop():
    pi.write(FL, 0)
    pi.write(BL, 0)
    pi.write(FR, 0)
    pi.write(BR, 0)
    pi.set_PWM_dutycycle(FL, 0)
    pi.set_PWM_dutycycle(FR, 0)
    pi.set_PWM_dutycycle(BL, 0)
    pi.set_PWM_dutycycle(BR, 0)

def avance_pwm(pwm_left, pwm_right):
    pi.write(BL, 0)
    pi.write(BR, 0)
    pi.set_PWM_dutycycle(FL, max(0, min(255, pwm_left)))
    pi.set_PWM_dutycycle(FR, max(0, min(255, pwm_right)))

def virage_gauche_pwm():
    pi.write(BR, 0)
    pi.write(FL, 0)
    pi.set_PWM_dutycycle(FR, PWM_VIRAGE)  # droite AVANT
    pi.set_PWM_dutycycle(BL, PWM_VIRAGE)  # gauche ARRÈRE

print("z : approche 10 cm + virage 90° GAUCHE précis, q : quitter\n")

try:
    while True:
        ch = getch()
        if ch == 'q':
            print("Quitter demandé.")
            break

        if ch == 'z':
            print("🚀 PHASE 1 : Approche 10 cm...")
            
            count_left = 0
            count_right = 0
            last_count_left = 0
            last_count_right = 0
            pwm_left = BASE_PWM_LEFT
            pwm_right = BASE_PWM_RIGHT

            while (count_left + count_right) // 2 < TICKS_APPROCHE:
                avance_pwm(pwm_left, pwm_right)
                time.sleep(PERIOD)

                dl = count_left - last_count_left
                dr = count_right - last_count_right
                last_count_left = count_left
                last_count_right = count_right

                diff = dl - dr
                corr = K_CORR * diff

                pwm_left  = BASE_PWM_LEFT  - corr
                pwm_right = BASE_PWM_RIGHT + corr

                pwm_left  = max(0, min(255, pwm_left))
                pwm_right = max(0, min(255, pwm_right))

            stop()
            print(f"  Approche OK : {count_left}/{count_right} ticks")

            print("🔄 PHASE 2 : Virage 90° GAUCHE...")
            
            count_left = 0
            count_right = 0
            
            while abs(count_left) < abs(TICKS_VIRAGE_GAUCHE) or count_right < TICKS_VIRAGE_DROITE:
                virage_gauche_pwm()
                time.sleep(PERIOD)

            stop()
            print(f"  Virage OK : G={count_left}, D={count_right} (ticks finaux)")
            print("✅ MISSION TERMINÉE")

except KeyboardInterrupt:
    print("\nArrêt par Ctrl+C...")

stop()
cbL.cancel()
cbR.cancel()
pi.stop()
print("Programme terminé.")