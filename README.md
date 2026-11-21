# Line-following robot

A two-wheeled line follower I built for a first-year robotics module at uni, where it won the Best Design award. It reads three IR sensors and steers along a black tape line. The code runs on the module's robot kit and uses its `robot` library for the sensors, motors, button and timer.

<p align="center">
  <img src="https://github.com/user-attachments/assets/545ee317-a541-4923-9dd9-ea18b19a8833" width="32%" />
  <img src="https://github.com/user-attachments/assets/ae9fa9c8-bde9-478f-90be-f5ac513b1b6b" width="32%" />
  <img src="https://github.com/user-attachments/assets/25be289d-64a6-470a-88b8-fd1c13d9ba67" width="32%" />
</p>

## Versions

I tried two approaches to steering, and later wrote a fixed-up third.

### `rule_based.py` (the faster one)

Each sensor is either on the line or off it, and each combination gets a fixed motor command:

| Sensors on the line | What it does |
|---|---|
| Middle only | Straight ahead at full speed |
| Left + middle / right + middle | Gentle turn: inner wheel slowed right down |
| Left only / right only | Sharp turn: inner wheel stopped |
| None | Line lost: keep turning the way it last turned |

The button toggles the robot on and off, and the loop runs at 150 Hz.

### `pid_hybrid.py`

Same idea for the extremes (full speed when centred, hard pivots for sharp turns, last-direction recovery when the line is lost), but small corrections in between come from a PID controller using `left − right` sensor reading as the error. Runs at 120 Hz.

## What I learned

The PID version never felt faster, and looking back at it I can see why:

- `set_motor_speeds()` clamps both wheels to `SHARP_TURN_SPEED` (18000), so the 40000 `CENTER_SPEED` could never actually be reached. The PID version was capped well below the rule-based one.
- The error is in raw sensor units (up to about 65 000), so with `Kp = 400` the correction is far bigger than the motor range and mostly just hits the limits.
- The integral has no limit, so it can wind up while the robot is pivoting.

### `pid_improved.py` (not tested yet)

`pid_hybrid.py` with those three things fixed:

- wheels are clamped to their own `MAX_SPEED`, so the robot can actually go fast
- the error is scaled to roughly −1…+1, so the gains are in sensible units
- the integral is capped and reset after pivots or losing the line

The gains are a starting point. To tune: set `Ki` and `Kd` to 0, raise `Kp` until it starts to wobble on the straights and back off a bit, then add `Kd` to calm the wobble, and only add a little `Ki` if it drifts to one side.

## Testing in a 2D sim

`sim/sim.py` runs any of the three files **unchanged** on a 2D track by pretending to be the `robot` library (motors, sensors, button, timer) with a simple two-wheel model. It's handy for comparing versions and trying gains before taking the robot out.

```
python sim/sim.py pid_improved.py   # watch it drive in a window
python sim/sim.py --compare         # run all three and print the results
```

On the sim track, 40 seconds each:

| Version | Laps | Avg speed | Mean distance from line |
|---|---|---|---|
| `rule_based.py` | 1.29 | 0.15 m/s | 3.5 mm |
| `pid_hybrid.py` | 0.88 | 0.10 m/s | 3.6 mm |
| `pid_improved.py` | 2.10 | 0.24 m/s | 2.1 mm |

The sim agrees with what happened on the real robot (the hybrid was slower than rule-based). The robot model is rough (wheel speeds, sensor spacing and motor lag are guesses in the top of the file), so the numbers are for comparing versions, not predicting real lap times.
