"""
Three analogue reflectance sensors on the Pico's ADC inputs.

Each sensor is normalised to 0..1 where 0 is the whitest and 1 the blackest
value seen during calibration. Normalising per sensor matters because the
three modules never give the same raw reading over the same surface.
"""

from machine import ADC, Pin
import time


class LineSensors:
    def __init__(self, left_pin, centre_pin, right_pin):
        self._adcs = (ADC(Pin(left_pin)), ADC(Pin(centre_pin)), ADC(Pin(right_pin)))
        # Sensible defaults in case the robot is started without calibrating.
        self._min = [8_000, 8_000, 8_000]
        self._max = [50_000, 50_000, 50_000]

    def read_raw(self):
        """Raw 16-bit readings (0..65535). Higher means darker for a TCRT5000 module."""
        return tuple(adc.read_u16() for adc in self._adcs)

    def calibrate(self, seconds, led=None):
        """
        Sample for a few seconds while the robot is slid back and forth over the
        line so every sensor sees both the white background and the black tape.
        """
        self._min = [65_535] * 3
        self._max = [0] * 3
        end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            for i, value in enumerate(self.read_raw()):
                if value < self._min[i]:
                    self._min[i] = value
                if value > self._max[i]:
                    self._max[i] = value
            if led is not None:
                led.toggle()
            time.sleep_ms(50)
        if led is not None:
            led.off()
        # Guard against a sensor that never saw any contrast.
        for i in range(3):
            if self._max[i] - self._min[i] < 2_000:
                self._max[i] = self._min[i] + 2_000

    def read(self):
        """Normalised readings (left, centre, right), each 0.0 (white) .. 1.0 (black)."""
        out = []
        for i, value in enumerate(self.read_raw()):
            span = self._max[i] - self._min[i]
            n = (value - self._min[i]) / span
            out.append(0.0 if n < 0 else 1.0 if n > 1 else n)
        return tuple(out)
