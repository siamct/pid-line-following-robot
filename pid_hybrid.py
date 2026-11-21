import robot

# --- Configuration Constants ---
SENSOR_THRESHOLD = 35000  # Threshold for detecting the black line
CENTER_SPEED = 40000      # Maximum speed when centered on the line
TURN_SPEED = 29000        # Speed for gradual turns
SHARP_TURN_SPEED = 18000  # Increased speed for sharper turns
LOST_SPEED = 18000
Kp = 400                  # Increased proportional gain for faster corrections
Ki = 0.02                 # Slightly higher integral gain for fine adjustments
Kd = 8                   # Increased derivative gain to dampen oscillations

# --- PID Controller State ---
previous_error = 0
integral = 0
last_turn_direction = None  # Tracks the last known turn direction ("left" or "right")

# --- Robot State ---
running = False
button_pressed_last = False

def stop_motors():
    """Stop all motors."""
    robot.motor[0].speed(0)
    robot.motor[1].speed(0)

def set_motor_speeds(left_speed, right_speed):
    """Clamp and set motor speeds."""
    left_speed = max(0, min(SHARP_TURN_SPEED, int(left_speed)))
    right_speed = max(0, min(SHARP_TURN_SPEED, int(right_speed)))
    robot.motor[0].speed(left_speed)
    robot.motor[1].speed(right_speed)

def calculate_pid(error):
    """Compute the PID correction based on the error."""
    global previous_error, integral

    # Proportional term
    P = Kp * error

    # Integral term (anti-windup)
    integral += error
    I = Ki * integral

    # Derivative term
    D = Kd * (error - previous_error)
    previous_error = error

    return P + I + D

def lost_track():
    """Behavior when the robot loses track of the line."""
    global last_turn_direction

    print("Line lost, continuing aggressive turn in the last known direction...")
    if last_turn_direction == "left":
        set_motor_speeds(0, SHARP_TURN_SPEED)  # Aggressively pivot right wheel
    elif last_turn_direction == "right":
        set_motor_speeds(SHARP_TURN_SPEED, 0)  # Aggressively pivot left wheel
    else:
        set_motor_speeds(LOST_SPEED, LOST_SPEED)  # Default to stopping if no direction is known

def process():
    """Main control loop for line following."""
    global last_turn_direction

    if not running:
        return

    # Read sensor values
    left_sensor = robot.sensor[5].read()
    middle_sensor = robot.sensor[3].read()
    right_sensor = robot.sensor[4].read()

    # Check if the robot has lost the track
    if left_sensor < SENSOR_THRESHOLD and middle_sensor < SENSOR_THRESHOLD and right_sensor < SENSOR_THRESHOLD:
        lost_track()
        return

    # Fast speed when perfectly centered on the black line
    if middle_sensor > SENSOR_THRESHOLD and left_sensor < SENSOR_THRESHOLD and right_sensor < SENSOR_THRESHOLD:
        set_motor_speeds(CENTER_SPEED, CENTER_SPEED)  # Full speed forward
        last_turn_direction = None  # Reset turn direction
        return

    # Sharp turn detection
    if left_sensor > SENSOR_THRESHOLD and middle_sensor < SENSOR_THRESHOLD and right_sensor < SENSOR_THRESHOLD:
        last_turn_direction = "left"  # Update last known turn direction
        set_motor_speeds(0, SHARP_TURN_SPEED)  # Aggressive left turn
        return
    elif right_sensor > SENSOR_THRESHOLD and middle_sensor < SENSOR_THRESHOLD and left_sensor < SENSOR_THRESHOLD:
        last_turn_direction = "right"  # Update last known turn direction
        set_motor_speeds(SHARP_TURN_SPEED, 0)  # Aggressive right turn
        return

    # PID-based adjustment for slight turns or corrections
    error = left_sensor - right_sensor
    correction = calculate_pid(error)
    left_speed = CENTER_SPEED - correction
    right_speed = CENTER_SPEED + correction
    set_motor_speeds(left_speed, right_speed)

    # Update last known direction for slight turns
    if correction > 0:
        last_turn_direction = "left"
    elif correction < 0:
        last_turn_direction = "right"

def check_button():
    """Toggle the robot state using a button."""
    global running, button_pressed_last
    button_pressed_now = robot.isButtonToggleOn()
    if button_pressed_now and not button_pressed_last:
        running = not running
        print("Robot " + ("ON" if running else "OFF"))
        if not running:
            stop_motors()
    button_pressed_last = button_pressed_now

# Timer setup to run the control loop regularly
robot.timer(frequency=120, callback=lambda: (check_button(), process()))


 