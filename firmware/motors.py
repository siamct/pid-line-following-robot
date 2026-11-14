"""
Two DC motors through a DRV8833 dual H-bridge.

Each motor has two inputs. Driving one with PWM and holding the other low
turns the motor one way; swap them for the other way. Both low is coast.
"""

from machine import PWM, Pin


class Motor:
    def __init__(self, pin_a, pin_b, frequency_hz):
        self._a = PWM(Pin(pin_a))
        self._b = PWM(Pin(pin_b))
        self._a.freq(frequency_hz)
        self._b.freq(frequency_hz)
        self.set(0.0)

    def set(self, speed):
        """speed in -1.0 .. +1.0; sign is direction."""
        speed = max(-1.0, min(1.0, speed))
        duty = int(abs(speed) * 65_535)
        if speed >= 0:
            self._a.duty_u16(duty)
            self._b.duty_u16(0)
        else:
            self._a.duty_u16(0)
            self._b.duty_u16(duty)

    def stop(self):
        self._a.duty_u16(0)
        self._b.duty_u16(0)


class Drive:
    """Left and right motors together."""

    def __init__(self, left_pins, right_pins, frequency_hz):
        self.left = Motor(left_pins[0], left_pins[1], frequency_hz)
        self.right = Motor(right_pins[0], right_pins[1], frequency_hz)

    def set(self, left_speed, right_speed):
        self.left.set(left_speed)
        self.right.set(right_speed)

    def stop(self):
        self.left.stop()
        self.right.stop()
