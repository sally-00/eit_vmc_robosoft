"""Calibrated shaft feedback from the existing single-owner ESP32 CSV recorder."""
import math


class ShaftDecoder:
    def __init__(self, calibration, filter_s=.05, observed_center=None):
        self.limit = math.radians(float(calibration['nominal_limit_deg']))
        spec = calibration['mapping']['base_x_deg']
        self.channel = spec['channel']
        self.center = float(spec['center_raw'])
        self.plus = float(spec['plus_limit_raw'])
        self.minus = float(spec['minus_limit_raw'])
        if (self.channel not in ('x_raw', 'y_raw') or
                not all(math.isfinite(x) and 0 <= x <= 4095 for x in
                        (self.center, self.plus, self.minus)) or
                (self.plus-self.center)*(self.minus-self.center) >= 0 or
                not math.isfinite(self.limit) or not 0 < self.limit < math.pi/2 or
                not math.isfinite(filter_s) or filter_s < 0):
            raise ValueError('Invalid Base-X shaft calibration or filter duration')
        self.offset = 0. if observed_center is None else float(observed_center)-self.center
        if not math.isfinite(self.offset):raise ValueError('Invalid observed upright reading')
        self.filter_s = filter_s
        self.previous = None
        self.rate = 0.

    def decode(self, row):
        stamp = int(row['host_unix_ns'])
        mono = int(row['host_monotonic_ns'])
        raw = int(row[self.channel])
        if stamp <= 0 or mono <= 0 or not 0 <= raw <= 4095:
            raise ValueError('Invalid shaft sample')
        corrected = raw-self.offset
        positive = (corrected-self.center)*(self.plus-self.center) >= 0
        endpoint = self.plus if positive else self.minus
        fraction = (corrected-self.center)/(endpoint-self.center)
        angle = self.limit*min(1., max(0., fraction))*(1. if positive else -1.)
        # At an ADC rail or beyond a calibrated endpoint the physical angle is
        # ambiguous. Publish invalid; never silently substitute the task command.
        clipped = raw in (0, 4095) or fraction >= 1.
        valid = not clipped
        if self.previous is not None:
            old_stamp, old_mono, old_angle = self.previous
            dt = (mono-old_mono)/1e9
            valid = valid and stamp > old_stamp and 0 < dt <= .5
            if valid:
                alpha = dt/(self.filter_s+dt)
                self.rate += alpha*((angle-old_angle)/dt-self.rate)
            else:
                self.rate = 0.
        else:
            # Two consecutive fresh samples are needed before using a rate.
            valid = False
        self.previous = stamp, mono, angle
        return dict(stamp_ns=stamp, angle_rad=angle,
                    angular_velocity_rad_s=self.rate,
                    measurement_valid=valid, clipped=clipped)
