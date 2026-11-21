import robot

# --- Configuration Constants ---
SENSOR_THRESHOLD = 45000      # Threshold for detecting the line (Unchanged)
MAX_MOTOR_SPEED = 27000       # Absolute maximum speed motors can be set to (Unchanged)
BASE_SPEED = 27000            # INCREASED: Slightly faster base speed
# Speed reduction for the inner wheel during turns when Middle+Side sensors are active.
# Set slightly less than BASE_SPEED for a very slow inner wheel roll during these turns.
# Increase to 17000 to make the inner wheel stop completely.
SLIGHT_TURN_INNER_REDUCTION = 26000
# Speed of the OUTER wheel during the sharpest turns (when only a side sensor is active, or off-track recovery)
# INCREASED to match the new BASE_SPEED.
SHARP_TURN_OUTER_SPEED = 20000

# --- Robot State ---
running = False
button_pressed_last = False
last_turn_direction = None   # Tracks the last turn direction ("left" or "right")

def stop_motors():
    """Stop motors completely."""
    robot.motor[0].speed(0)
    robot.motor[1].speed(0)

def set_motor_speeds(left_speed, right_speed):
    """Clamp speeds using MAX_MOTOR_SPEED and set motor speeds."""
    # Ensure speeds are within 0 and MAX_MOTOR_SPEED
    left_speed = max(0, min(MAX_MOTOR_SPEED, int(left_speed)))
    right_speed = max(0, min(MAX_MOTOR_SPEED, int(right_speed)))
    robot.motor[0].speed(left_speed)
    robot.motor[1].speed(right_speed)

def process():
    """Main control loop: Reads sensors and adjusts motor speeds."""
    global last_turn_direction
    if not running:
        return

    # Read sensor values
    left_sensor = robot.sensor[5].read()
    middle_sensor = robot.sensor[3].read()
    right_sensor = robot.sensor[4].read()

    # Determine boolean detection state
    left_detected = left_sensor > SENSOR_THRESHOLD
    middle_detected = middle_sensor > SENSOR_THRESHOLD
    right_detected = right_sensor > SENSOR_THRESHOLD

    # Determine motion logic
    if middle_detected and not left_detected and not right_detected:
        # Go forward (Middle only) - Use the increased BASE_SPEED
        set_motor_speeds(BASE_SPEED, BASE_SPEED)
        last_turn_direction = None  # Reset last turn direction

    elif left_detected and middle_detected:
        # Turn Right (Left + Middle detected) - Slow inner (left) wheel significantly
        # Inner speed = 17000 - 16000 = 1000. Outer speed = 17000.
        set_motor_speeds(BASE_SPEED - SLIGHT_TURN_INNER_REDUCTION, BASE_SPEED)
        last_turn_direction = "right"

    elif right_detected and middle_detected:
        # Turn Left (Right + Middle detected) - Slow inner (right) wheel significantly
        # Inner speed = 1000. Outer speed = 17000.
        set_motor_speeds(BASE_SPEED, BASE_SPEED - SLIGHT_TURN_INNER_REDUCTION)
        last_turn_direction = "left"

    elif left_detected and not right_detected:
        # Sharp turn Left (Left Only detected) - Stop inner (left), Outer (right) at SHARP_TURN_OUTER_SPEED
        set_motor_speeds(0, SHARP_TURN_OUTER_SPEED)
        last_turn_direction = "left"

    elif right_detected and not left_detected:
        # Sharp turn Right (Right Only detected) - Stop inner (right), Outer (left) at SHARP_TURN_OUTER_SPEED
        set_motor_speeds(SHARP_TURN_OUTER_SPEED, 0)
        last_turn_direction = "right"

    else:
        # Off Track - Line Lost Recovery
        # Continue the last known turn direction aggressively using the increased SHARP_TURN_OUTER_SPEED
        if last_turn_direction == "left":
            set_motor_speeds(0, SHARP_TURN_OUTER_SPEED)  # Aggressive turn left
        elif last_turn_direction == "right":
            set_motor_speeds(SHARP_TURN_OUTER_SPEED, 0)  # Aggressive turn right
        else:
            # Default search turn left if unsure.
            set_motor_speeds(0, SHARP_TURN_OUTER_SPEED) # Default search turn

def check_button():
    """Toggle robot state based on button press."""
    global running, button_pressed_last
    button_pressed_now = robot.isButtonToggleOn()
    if button_pressed_now and not button_pressed_last:
        running = not running
        print("Robot " + ("ON" if running else "OFF"))
        if not running:
            stop_motors()
    button_pressed_last = button_pressed_now

# Timer setup for regular updates (150Hz is likely okay)
robot.timer(frequency=150, callback=lambda: (check_button(), process()))
 