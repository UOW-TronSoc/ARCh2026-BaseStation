#!/usr/bin/env python3
"""
NIR servo on BOARD pin 15: PWM at 50 Hz.

Interactive control is handled in-process by Django via POST /api/nir-servo-control/
which accepts {"duty": 0-100} and applies it in real time (arrow-key driven from the
Science page in the browser).

Running this script standalone performs the original 0%→60%→0% demo sweep.
"""
import time

import Jetson.GPIO as GPIO

SERVO_PIN = 15
PWM_FREQ = 50
STEP_DELAY_S = 0.03

if __name__ == "__main__":
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
    pwm.start(0)

    try:
        print("Ramp PWM duty 0% -> 60% ...")
        for duty in range(0, 61):
            pwm.ChangeDutyCycle(duty)
            time.sleep(STEP_DELAY_S)
            print(f"  duty={duty}%")

        print("Ramp PWM duty 60% -> 0% ...")
        for duty in range(60, -1, -1):
            pwm.ChangeDutyCycle(duty)
            time.sleep(STEP_DELAY_S)
            print(f"  duty={duty}%")
        print("Done.")
    finally:
        pwm.ChangeDutyCycle(0)
        pwm.stop()
        GPIO.cleanup(SERVO_PIN)
