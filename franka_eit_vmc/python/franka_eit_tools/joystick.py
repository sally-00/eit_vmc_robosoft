"""Calibrated shaft feedback from the existing single-owner ESP32 CSV recorder."""
import math


class ShaftDecoder:
    def __init__(self, calibration, filter_s=.05, observed_center=None, observed_center_y=None):
        self.limit = math.radians(float(calibration['nominal_limit_deg']))
        spec = calibration['mapping']['base_x_deg']
        spec_y = calibration['mapping']['base_y_deg']
        self.channel = spec['channel']
        self.channel_y = spec_y['channel']
        self.center = float(spec['center_raw'])
        self.plus = float(spec['plus_limit_raw'])
        self.minus = float(spec['minus_limit_raw'])
        self.center_y = float(spec_y['center_raw'])
        self.plus_y = float(spec_y['plus_limit_raw'])
        self.minus_y = float(spec_y['minus_limit_raw'])
        if (self.channel not in ('x_raw', 'y_raw') or
                self.channel_y not in ('x_raw', 'y_raw') or self.channel_y == self.channel or
                not all(math.isfinite(x) and 0 <= x <= 4095 for x in
                        (self.center, self.plus, self.minus,
                         self.center_y, self.plus_y, self.minus_y)) or
                (self.plus-self.center)*(self.minus-self.center) >= 0 or
                (self.plus_y-self.center_y)*(self.minus_y-self.center_y) >= 0 or
                not math.isfinite(self.limit) or not 0 < self.limit < math.pi/2 or
                not math.isfinite(filter_s) or filter_s < 0):
            raise ValueError('Invalid two-axis shaft calibration or filter duration')
        self.offset = 0. if observed_center is None else float(observed_center)-self.center
        self.offset_y = 0. if observed_center_y is None else float(observed_center_y)-self.center_y
        if not math.isfinite(self.offset) or not math.isfinite(self.offset_y):
            raise ValueError('Invalid observed upright reading')
        self.filter_s = filter_s
        self.previous = None
        self.rate = 0.
        self.rate_y = 0.

    def _angle(self, raw, center, plus, minus, offset):
        corrected = raw-offset
        positive = (corrected-center)*(plus-center) >= 0
        endpoint = plus if positive else minus
        fraction = (corrected-center)/(endpoint-center)
        return (self.limit*min(1., max(0., fraction))*(1. if positive else -1.),
                raw in (0, 4095) or fraction >= 1.)

    def decode(self, row):
        stamp = int(row['host_unix_ns'])
        mono = int(row['host_monotonic_ns'])
        raw = int(row[self.channel])
        raw_y = int(row[self.channel_y])
        if stamp <= 0 or mono <= 0 or not 0 <= raw <= 4095 or not 0 <= raw_y <= 4095:
            raise ValueError('Invalid shaft sample')
        angle, clipped_x = self._angle(raw,self.center,self.plus,self.minus,self.offset)
        angle_y, clipped_y = self._angle(raw_y,self.center_y,self.plus_y,self.minus_y,self.offset_y)
        # Keep the rail flag for analysis, but use the bounded endpoint angle
        # for live control when either channel reaches its ADC limit.
        clipped = clipped_x or clipped_y
        valid = True
        if self.previous is not None:
            old_stamp, old_mono, old_angle, old_angle_y = self.previous
            dt = (mono-old_mono)/1e9
            valid = valid and stamp > old_stamp and 0 < dt <= .5
            if valid:
                alpha = dt/(self.filter_s+dt)
                self.rate += alpha*((angle-old_angle)/dt-self.rate)
                self.rate_y += alpha*((angle_y-old_angle_y)/dt-self.rate_y)
            else:
                self.rate = 0.
                self.rate_y = 0.
        else:
            # Two consecutive fresh samples are needed before using a rate.
            valid = False
        self.previous = stamp, mono, angle, angle_y
        return dict(stamp_ns=stamp, angle_rad=angle,
                    angular_velocity_rad_s=self.rate,
                    angle_y_rad=angle_y, angular_velocity_y_rad_s=self.rate_y,
                    measurement_valid=valid, clipped=clipped)
