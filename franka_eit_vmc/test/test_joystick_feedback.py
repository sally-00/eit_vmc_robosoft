"""Calibration, freshness and rail rejection without a serial device or robot."""
import json
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from franka_eit_tools.joystick import ShaftDecoder


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.cal=json.loads((Path(__file__).resolve().parents[1]/'config/joystick_adc_calibration.json').read_text())

    def row(self, raw, index=1):
        return dict(host_unix_ns=str(1_000_000_000+index*10_000_000),
                    host_monotonic_ns=str(10_000_000_000+index*10_000_000),
                    x_raw='2043',y_raw=str(raw))

    def test_center_and_direction_follow_base_x_mapping(self):
        d=ShaftDecoder(self.cal)
        self.assertFalse(d.decode(self.row(1987))['measurement_valid'])
        zero=d.decode(self.row(1987,2));self.assertTrue(zero['measurement_valid'])
        self.assertEqual(zero['angle_rad'],0.)
        positive=d.decode(self.row(1000,3));negative=d.decode(self.row(3000,4))
        self.assertGreater(positive['angle_rad'],0.)
        self.assertLess(negative['angle_rad'],0.)
        self.assertAlmostEqual(positive['angle_rad'],math.radians(35)*987/1987)

    def test_clipped_samples_are_invalid_instead_of_command_fallback(self):
        d=ShaftDecoder(self.cal)
        d.decode(self.row(1987))
        for i,raw in enumerate((0,4095),2):
            result=d.decode(self.row(raw,i))
            self.assertTrue(result['clipped']);self.assertFalse(result['measurement_valid'])

    def test_timestamp_is_not_refreshed_by_decoding(self):
        d=ShaftDecoder(self.cal)
        row=self.row(1987)
        result=d.decode(row)
        self.assertEqual(result['stamp_ns'],int(row['host_unix_ns']))
        self.assertFalse(d.decode(row)['measurement_valid'])
        self.assertFalse(d.decode(self.row(1987,100))['measurement_valid'])

    def test_velocity_uses_elapsed_sample_time(self):
        d=ShaftDecoder(self.cal,filter_s=0.)
        d.decode(self.row(1987))
        result=d.decode(self.row(1900,2))
        self.assertAlmostEqual(result['angular_velocity_rad_s'],result['angle_rad']/.01)

    def test_reversed_adc_direction_is_supported(self):
        self.cal['mapping']['base_x_deg'].update(plus_limit_raw=4095,minus_limit_raw=0)
        d=ShaftDecoder(self.cal)
        d.decode(self.row(1987));self.assertGreater(d.decode(self.row(3000,2))['angle_rad'],0.)

    def test_observed_upright_override_is_used_by_live_angle(self):
        d=ShaftDecoder(self.cal,observed_center=2000)
        d.decode(self.row(2000))
        self.assertEqual(d.decode(self.row(2000,2))['angle_rad'],0.)
        self.assertTrue(d.decode(self.row(0,3))['clipped'])

    def test_invalid_calibration_and_values_are_rejected(self):
        self.cal['mapping']['base_x_deg']['center_raw']=0
        with self.assertRaises(ValueError):ShaftDecoder(self.cal)
        self.setUp();d=ShaftDecoder(self.cal)
        with self.assertRaises(ValueError):d.decode(self.row(4096))
        with self.assertRaises(ValueError):ShaftDecoder(self.cal,filter_s=float('nan'))

if __name__=='__main__':unittest.main()
