"""
Pico W line follower: main program.

Sequence
    1. Power on. LED off. Waiting for the button.
    2. Press the button: LED blinks while the sensors calibrate. Slide the
       robot sideways across the line a few times during this time.
    3. LED on steady: calibrated. Place the robot on the line and press the
       button again to start.
    4. Running. Press the button again to stop.

Readings and the PID terms are printed over USB serial every 50 loops so
the gains can be tuned from Thonny without reflashing.
"""

import time
from machine import Pin

import config
from controller import PID, line_position, mix
from motors import Drive
from sensors import LineSensors


def wait_for_press(button):
    """Block until the button is pressed and released, with simple debounce."""
    while button.value() == 1:
        time.sleep_ms(10)
    time.sleep_ms(30)
    while button.value() == 0:
        time.sleep_ms(10)
    time.sleep_ms(30)


def button_pressed(button):
    return button.value() == 0


def main():
    led = Pin(config.LED_PIN, Pin.OUT)
    button = Pin(config.BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    sensors = LineSensors(config.SENSOR_LEFT_PIN, config.SENSOR_CENTRE_PIN, config.SENSOR_RIGHT_PIN)
    drive = Drive(config.MOTOR_LEFT_PINS, config.MOTOR_RIGHT_PINS, config.PWM_FREQUENCY_HZ)
    pid = PID(config.KP, config.KI, config.KD, integral_limit=config.INTEGRAL_LIMIT)

    led.off()
    print("Press the button to calibrate.")
    wait_for_press(button)

    print("Calibrating: slide the robot across the line...")
    sensors.calibrate(config.CALIBRATION_SECONDS, led)
    led.on()
    print("Calibrated. Place on the line and press the button to start.")
    wait_for_press(button)

    pid.reset()
    last_error = 0.0
    dt = config.LOOP_PERIOD_MS / 1000.0
    loops = 0
    print("Running.")

    try:
        while True:
            start = time.ticks_ms()

            left, centre, right = sensors.read()
            error, seen = line_position(left, centre, right, config.LINE_THRESHOLD, last_error)
            if seen:
                last_error = error

            correction = pid.update(error, dt)
            left_speed, right_speed = mix(config.BASE_SPEED, correction, config.MAX_SPEED)
            drive.set(left_speed, right_speed)

            loops += 1
            if loops % 50 == 0:
                print("sens=%.2f %.2f %.2f  err=%+.2f  P=%+.2f I=%+.2f D=%+.2f  L=%.2f R=%.2f%s" % (
                    left, centre, right, error, pid.last_p, pid.last_i, pid.last_d,
                    left_speed, right_speed, "" if seen else "  LOST"))

            if button_pressed(button):
                break

            # Keep the loop period fixed so the PID derivative and integral are consistent.
            elapsed = time.ticks_diff(time.ticks_ms(), start)
            if elapsed < config.LOOP_PERIOD_MS:
                time.sleep_ms(config.LOOP_PERIOD_MS - elapsed)
    finally:
        drive.stop()
        led.off()
        print("Stopped.")


main()
