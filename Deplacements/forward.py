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

BASE_PWM_LEFT  = 155      # +5 pour compenser dérive droite
BASE_PWM_RIGHT = 150
K_CORR = 8                # correction renforcée
PERIOD = 0.03
TICKS_TARGET = 400        # ≈ 15-18 cm

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

def avance_pwm(pwm_left, pwm_right):
    pi.write(BL, 0)
    pi.write(BR, 0)
    pi.set_PWM_dutycycle(FL, max(0, min(255, pwm_left)))
    pi.set_PWM_dutycycle(FR, max(0, min(255, pwm_right)))

print("z : avancer exactement 15-18 cm, q : quitter\n")

pwm_left = BASE_PWM_LEFT
pwm_right = BASE_PWM_RIGHT

try:
    while True:
        ch = getch()
        if ch == 'q':
            print("Quitter demandé.")
            break

        if ch == 'z':
            print(f"🚀 Avance (cible {TICKS_TARGET} ticks)...")

            count_left = 0
            count_right = 0
            last_count_left = 0
            last_count_right = 0

            while (count_left + count_right) // 2 < TICKS_TARGET:
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
            print(f"✅ Arrêt après {count_left} ticks G, {count_right} ticks D (moyenne {(count_left + count_right)//2})")

except KeyboardInterrupt:
    print("\nArrêt par Ctrl+C...")

stop()
cbL.cancel()
cbR.cancel()
pi.stop()
print("Programme terminé.")
