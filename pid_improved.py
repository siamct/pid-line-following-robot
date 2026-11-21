import robot

# pid_hybrid.py with the problems fixed (see README). Not tested on the robot yet,
# so the gains below are a starting point, not tuned values.

# --- Sensors ---
SENSOR_THRESHOLD = 35000   # above this = sensor is over the black line
SENSOR_MAX = 65535         # raw reading range, used to scale the error

# --- Speeds ---
MAX_SPEED = 32000          # hard limit for either wheel (was SHARP_TURN_SPEED by mistake)
BASE_SPEED = 27000         # forward speed when following normally
SHARP_TURN_SPEED = 20000   # outer wheel speed when pivoting
LOST_SPEED = 15000         # creep forward if the line is lost with no known direction

# --- PID gains (error is scaled to roughly -1..+1) ---
Kp = 18000                 # wheel-speed change for a full-scale error
Ki = 40                    # small: only there to trim out a steady drift
Kd = 3000                  # damps the wobble
INTEGRAL_LIMIT = 100       # stops the integral winding up during pivots

# --- State ---
previous_error = 0
integral = 0
last_turn_direction = None
running = False
button_pressed_last = False


def clamp(value, low, high):
    return max(low, min(high, value))


def stop_motors():
    robot.motor[0].speed(0)
    robot.motor[1].speed(0)


def set_motor_speeds(left_speed, right_speed):
    robot.motor[0].speed(int(clamp(left_speed, 0, MAX_SPEED)))
    robot.motor[1].speed(int(clamp(right_speed, 0, MAX_SPEED)))


def reset_pid():
    global previous_error, integral
    previous_error = 0
    integral = 0


def calculate_pid(error):
    global previous_error, integral
    integral = clamp(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
    derivative = error - previous_error
    previous_error = error
    return Kp * error + Ki * integral + Kd * derivative


def process():
    global last_turn_direction

    if not running:
        return

    left = robot.sensor[5].read()
    middle = robot.sensor[3].read()
    right = robot.sensor[4].read()

    left_on = left > SENSOR_THRESHOLD
    middle_on = middle > SENSOR_THRESHOLD
    right_on = right > SENSOR_THRESHOLD

    # Line lost: keep turning the way we were going
    if not (left_on or middle_on or right_on):
        reset_pid()
        if last_turn_direction == "left":
            set_motor_speeds(0, SHARP_TURN_SPEED)
        elif last_turn_direction == "right":
            set_motor_speeds(SHARP_TURN_SPEED, 0)
        else:
            set_motor_speeds(LOST_SPEED, LOST_SPEED)
        return

    # Only an outside sensor sees the line: too far off for PID, pivot
    if left_on and not middle_on and not right_on:
        reset_pid()
        last_turn_direction = "left"
        set_motor_speeds(0, SHARP_TURN_SPEED)
        return
    if right_on and not middle_on and not left_on:
        reset_pid()
        last_turn_direction = "right"
        set_motor_speeds(SHARP_TURN_SPEED, 0)
        return

    # Otherwise steer smoothly with PID.
    # Positive error = line is towards the left sensor = slow the left wheel.
    error = (left - right) / SENSOR_MAX
    correction = calculate_pid(error)
    set_motor_speeds(BASE_SPEED - correction, BASE_SPEED + correction)

    if correction > 0:
        last_turn_direction = "left"
    elif correction < 0:
        last_turn_direction = "right"


def check_button():
    global running, button_pressed_last
    pressed = robot.isButtonToggleOn()
    if pressed and not button_pressed_last:
        running = not running
        print("Robot " + ("ON" if running else "OFF"))
        if not running:
            stop_motors()
            reset_pid()
    button_pressed_last = pressed


robot.timer(frequency=120, callback=lambda: (check_button(), process()))
