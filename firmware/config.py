"""
Pin assignments and tuning constants for the Pico W line follower.

Change this file rather than the logic when moving to a different chassis.
"""

# ----- Sensors: three TCRT5000 reflectance modules, analogue outputs -----
# The Pico has exactly three external ADC inputs, GP26, GP27 and GP28.
SENSOR_LEFT_PIN = 26      # ADC0
SENSOR_CENTRE_PIN = 27    # ADC1
SENSOR_RIGHT_PIN = 28     # ADC2

# ----- Motors: DRV8833 dual H-bridge -----
# Each motor uses two PWM pins (IN1, IN2). Swap the pair if a motor runs backwards.
MOTOR_LEFT_PINS = (2, 3)
MOTOR_RIGHT_PINS = (4, 5)
PWM_FREQUENCY_HZ = 10_000  # above audible range so the motors do not whine

# ----- Button and LED -----
BUTTON_PIN = 14           # momentary button to GND, internal pull-up
LED_PIN = "LED"           # the Pico W on-board LED

# ----- Control loop -----
LOOP_PERIOD_MS = 10       # 100 Hz

# PID gains. Error is in the range -1..+1, output is a wheel-speed correction
# in the range -1..+1. These were found by trial and error on the final track
# and will need retuning for a different chassis, speed or line width.
KP = 0.80
KI = 0.20
KD = 0.06
INTEGRAL_LIMIT = 0.5

BASE_SPEED = 0.60         # forward speed on a straight, fraction of full duty
MAX_SPEED = 0.80          # cap to keep the robot controllable on 3 x AA cells

# Below this total normalised signal the line is considered lost.
LINE_THRESHOLD = 0.30

# Seconds the robot waits for you to sweep it across the line during calibration.
CALIBRATION_SECONDS = 5
