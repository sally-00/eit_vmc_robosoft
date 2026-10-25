#!/usr/bin/env python3
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from franka_eit_tools.position_model import PositionModel
from franka_eit_tools.inference import Inference
from franka_eit_tools.model_selection import select_model
from franka_eit_tools.eit_reader import EITReader, measurement_protocol, parse_frame
from franka_eit_tools.mount import pair_yaw_for_base_x, ready_pad_rotation, number_angle

MODEL = ROOT / 'models/position_model.npz'


class Tests(unittest.TestCase):
    def test_selects_newest_trained_checkpoint(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            older = root / 'older' / 'position_model.npz'
            newer = root / 'newer' / 'position_model.npz'
            older.parent.mkdir(); newer.parent.mkdir()
            older.write_bytes(b'old'); newer.write_bytes(b'new')
            os.utime(older, ns=(100, 100))
            os.utime(newer, ns=(200, 200))
            self.assertEqual(select_model(runs_dir=root), newer)
            os.utime(older, ns=(300, 300))
            self.assertEqual(select_model(runs_dir=root), older)
            self.assertEqual(select_model(newer, runs_dir=root), newer)

    def test_model_scan_and_training_predictions(self):
        model = PositionModel.load(MODEL)
        self.assertEqual(model.channels, 304)
        self.assertEqual(model.x.shape, (40, 608))
        self.assertEqual(len(model.protocol), 304)
        self.assertEqual(model.metadata['eit_config']['eit_drive_pattern'], 'combined')
        self.assertTrue(model.allow_clipped)
        expected = {0: [0.03132909210833823, -0.3051030088828455],
                    1: [-6.860636116234872, 6.5100508184072865],
                    10: [12.513366757742135, -26.303813672452236],
                    39: [-1.75273750250944, 39.63006474328955]}
        for index, xy in expected.items():
            np.testing.assert_allclose(model.estimate(model.x[index])[0][0], xy, atol=1e-10)

    def test_protocol_and_frame(self):
        pins = list(range(1, 17))
        protocol = measurement_protocol(pins, 'combined')
        self.assertEqual(len(protocol), 304)
        self.assertEqual(protocol[0], (1, 2, 3, 4))
        self.assertEqual(len(measurement_protocol(list(range(1, 9)), 'adjacent')), 40)
        line = ','.join(['0.01|0'] * 303 + ['0.02|0.1C'])
        self.assertTrue(parse_frame(line, len(protocol))['clipped'][-1])
        with self.assertRaises(ValueError):
            parse_frame(line, 40)

    def test_reader_uses_saved_acquisition_settings(self):
        model = PositionModel.load(MODEL)
        saved = model.metadata['eit_config']
        reader = EITReader(saved)
        self.assertEqual(reader.protocol, model.protocol)
        self.assertEqual(reader.drive_pattern, 'combined')
        self.assertEqual(reader.rx_gain_margin_steps, 2)
        self.assertEqual(reader.frequency, 50)

    def test_reader_releases_port_when_stop_command_fails(self):
        model = PositionModel.load(MODEL)
        reader = EITReader(model.metadata['eit_config'])
        port = Mock()
        port.write.side_effect = OSError('USB disconnected')
        reader.ser = port
        reader.close()
        port.close.assert_called_once_with()
        self.assertIsNone(reader.ser)

    def test_direct_inference_and_hysteresis(self):
        engine = Inference(MODEL)
        model = engine.model
        self.assertTrue(engine.physical)
        channels = model.channels
        base = {'magnitude_Vrms': [1.] * channels, 'phase_rad': [0.] * channels,
                'clipped': [False] * channels}
        engine.set_baseline({'frames': [base] * 5})
        features = model.x[0]
        voltage = 1 + .5 * (features[:channels] + 1j * features[channels:])
        frame = {'magnitude_Vrms': np.abs(voltage).tolist(),
                 'phase_rad': np.angle(voltage).tolist(), 'clipped': [False] * channels}
        for _ in range(3):
            output = engine.step(frame)
        self.assertTrue(output['contact'])
        self.assertTrue(output['position_valid'])
        self.assertEqual(output['contact_threshold'], 0.0)
        np.testing.assert_allclose([output['x_m'], output['y_m']],
                                   model.estimate(features)[0][0] / 1000, atol=1e-10)
        engine.set_contact_threshold(output['signal_strength'] * 1.1)
        for _ in range(2):
            output = engine.step(frame)
        self.assertFalse(output['contact'])
        self.assertEqual(output['contact_threshold'],engine.contact_threshold)
        with self.assertRaises(ValueError):engine.set_contact_threshold(-1.)
        engine.set_contact_threshold(0.)
        limited = Inference(MODEL, contact_threshold=output['signal_strength'] * 1.1)
        limited.set_baseline({'frames': [base] * 5})
        for _ in range(3):
            limited_output = limited.step(frame)
        self.assertFalse(limited_output['contact'])
        self.assertTrue(limited_output['measurement_valid'])
        for _ in range(2):
            output = engine.step(base)
        self.assertFalse(output['contact'])
        self.assertFalse(output['position_valid'])
        frame['clipped'] = [True] * channels
        self.assertTrue(engine.step(frame)['measurement_valid'])  # Saved model allows clipping.

    def test_invalid_checkpoint(self):
        with np.load(MODEL, allow_pickle=False) as data:
            arrays = {key: data[key] for key in data.files}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'model.npz'
            np.savez(path, **{**arrays, 'x': np.ones((40, 80))})
            with self.assertRaises(ValueError):
                PositionModel.load(path)
            np.savez(path, **{**arrays, 'x': np.array([object()], dtype=object)})
            with self.assertRaises(ValueError):
                PositionModel.load(path)

    def test_pair_mapping(self):
        for numbering in ['ccw', 'cw']:
            for first in range(1, 9):
                second = first % 8 + 1
                angle = pair_yaw_for_base_x([first, second], numbering)
                alpha, beta = number_angle(first, numbering), number_angle(second, numbering)
                vector = np.array([math.cos(alpha) + math.cos(beta),
                                   math.sin(alpha) + math.sin(beta), 0.])
                vector /= np.linalg.norm(vector)
                self.assertTrue(np.allclose(ready_pad_rotation(angle) @ vector, [1, 0, 0]))


if __name__ == '__main__':
    unittest.main()
