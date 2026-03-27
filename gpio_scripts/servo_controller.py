#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time

# Use BOARD numbering so this matches the physical header pin number
SERVO_PIN = 33          # change this if your servo is actually on a different header pin
PWM_FREQ = 50           # 50 Hz for hobby servos
START_ANGLE = 10
TARGET_ANGLE = 120

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
pwm.start(7.5)          # centre position

def angle_to_duty(angle_deg: float) -> float:
    """
    Map 0 to 180 degrees to duty cycle for a typical servo at 50 Hz.
    0 deg   -> about 2.5%
    90 deg  -> about 7.5%
    180 deg -> about 12.5%
    """
    angle_deg = max(0.0, min(180.0, angle_deg))
    return 2.5 + (angle_deg / 180.0) * 10.0

def set_angle(angle_deg: float, settle_time: float = 0.5):
    duty = angle_to_duty(angle_deg)
    pwm.ChangeDutyCycle(duty)
    time.sleep(settle_time)

try:
    print(f"Moving from {START_ANGLE} to {TARGET_ANGLE} degrees slowly...")
    for angle in range(START_ANGLE, TARGET_ANGLE + 1):
        duty = angle_to_duty(angle)
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.05)
        print(f"  {angle} deg")

    print(f"Moving back from {TARGET_ANGLE} to {START_ANGLE} degrees slowly...")
    for angle in range(TARGET_ANGLE, START_ANGLE - 1, -1):
        duty = angle_to_duty(angle)
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.05)
        print(f"  {angle} deg")
    print("Done.")

finally:
    pwm.ChangeDutyCycle(0)
    pwm.stop()
    GPIO.cleanup()