#!/usr/bin/env python3
import json
import hashlib
import importlib.util
import threading
from types import SimpleNamespace
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

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

    def test_sensor_selects_scan_from_model(self):
        script=ROOT/'scripts/eit_contact_node.py'
        spec=importlib.util.spec_from_file_location('eit_contact_scan_test',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        model=PositionModel.load(MODEL)
        host=json.loads((ROOT/'config/eit.json').read_text())
        host.update(eit_port='/dev/ttyACM0',contact_threshold=.25,
                    eit_drive_pattern='adjacent',eit_frequency_khz=20)
        effective,reader=module.acquisition_config_for_model(host,model)
        self.assertEqual(effective['eit_drive_pattern'],'combined')
        self.assertEqual(effective['eit_frequency_khz'],50)
        self.assertEqual(effective['eit_port'],'/dev/ttyACM0')
        self.assertEqual(effective['contact_threshold'],.25)
        self.assertEqual(reader.protocol,model.protocol)
        self.assertEqual(len(reader.protocol),304)
        self.assertEqual(host['eit_drive_pattern'],'adjacent')

        adjacent_settings=dict(model.metadata['eit_config'],eit_drive_pattern='adjacent')
        adjacent_model=SimpleNamespace(metadata={'eit_config':adjacent_settings},
            protocol=measurement_protocol(adjacent_settings['eit_electrode_pins'],'adjacent'),
            channels=208)
        adjacent,reader=module.acquisition_config_for_model(host,adjacent_model)
        self.assertEqual(adjacent['eit_drive_pattern'],'adjacent')
        self.assertEqual(len(reader.protocol),208)
        with self.assertRaisesRegex(ValueError,'different measurement channels'):
            module.acquisition_config_for_model(host,SimpleNamespace(
                metadata=adjacent_model.metadata,protocol=model.protocol,channels=304))

    def test_sensor_replaces_only_its_previous_port_owner(self):
        script=ROOT/'scripts/eit_contact_node.py'
        spec=importlib.util.spec_from_file_location('eit_contact_takeover_test',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        prior=['python3','/home/birl/franka_ros2_ws/install/franka_eit_vmc/lib/franka_eit_vmc/eit_contact_node.py']
        logger=Mock()
        with patch.object(module,'serial_port_owners',side_effect=[[123],[123],[]]), \
             patch.object(module,'robot_controller_running',return_value=False), \
             patch.object(module,'process_arguments',return_value=prior), \
             patch.object(module.Path,'stat',return_value=SimpleNamespace(st_uid=os.getuid())), \
             patch.object(module.os,'kill') as kill:
            module.release_previous_eit_node('/dev/ttyACM0',logger)
        kill.assert_called_once_with(123,module.signal.SIGINT)
        logger.info.assert_called_once()
        with patch.object(module,'serial_port_owners',return_value=[456]), \
             patch.object(module,'robot_controller_running',return_value=False), \
             patch.object(module,'process_arguments',return_value=['python3','contact_position_app.py']), \
             patch.object(module.Path,'stat',return_value=SimpleNamespace(st_uid=os.getuid())), \
             patch.object(module.os,'kill') as kill:
            with self.assertRaisesRegex(RuntimeError,'held by PID 456'):
                module.release_previous_eit_node('/dev/ttyACM0',logger)
        kill.assert_not_called()
        with patch.object(module,'serial_port_owners',return_value=[123]), \
             patch.object(module,'robot_controller_running',return_value=True), \
             patch.object(module.os,'kill') as kill:
            with self.assertRaisesRegex(RuntimeError,'controller is running'):
                module.release_previous_eit_node('/dev/ttyACM0',logger)
        kill.assert_not_called()

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
        self.assertTrue(output['contact_candidate'])
        self.assertTrue(output['position_valid'])
        self.assertLessEqual(output['pattern_distance'],output['pattern_support_limit'])
        self.assertEqual(output['contact_threshold'], 0.0)
        np.testing.assert_allclose([output['x_m'], output['y_m']],
                                   model.estimate(features)[0][0] / 1000, atol=1e-10)
        engine.set_contact_threshold(output['signal_strength'] * 1.1)
        for _ in range(2):
            output = engine.step(frame)
        self.assertFalse(output['contact'])
        self.assertFalse(output['contact_candidate'])
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

    def test_startup_baseline_calibrates_once_after_a_connection_retry(self):
        script=ROOT/'scripts/eit_contact_node.py'
        spec=importlib.util.spec_from_file_location('eit_contact_startup_test',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        config=ROOT/'config/eit.json'
        settings=json.loads(config.read_text())
        class Stop:
            done=False
            def is_set(self):return self.done
            def wait(self,seconds):return self.done
        stop=Stop()
        class Reader:
            def __init__(self,settings):
                self.protocol=measurement_protocol(settings['eit_electrode_pins'],
                                                    settings['eit_drive_pattern'])
                self.ser=SimpleNamespace(in_waiting=0)
                self._rx_buffer=b'';self.opens=0;self.count=0
            def open(self):self.opens+=1;self.count=0
            def close(self):pass
            def _line(self,deadline):
                self.count+=1
                if self.opens==1 and self.count==8:raise OSError('interrupted startup frame')
                if self.opens==2 and self.count==25:stop.done=True
                return f'{self.opens}:{self.count}'
        class Engine:
            def __init__(self,path,contact_threshold):
                self.sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest()
                protocol=measurement_protocol(settings['eit_electrode_pins'],settings['eit_drive_pattern'])
                self.model=SimpleNamespace(metadata={'eit_config':settings},support_limit=1.,
                                           protocol=protocol,channels=len(protocol))
                self.physical=True;self.baseline=None;self.on=self.off=0;self.contact=False
                self.contact_threshold=contact_threshold;self.captures=[]
            def set_baseline(self,value):
                self.captures.append(value);self.baseline=value
            def step(self,frame):return {}
        class Thread:
            def __init__(self,target,daemon):self.target=target
            def start(self):pass
        args=SimpleNamespace(model=None,config=None,frame='eit_pad')
        with patch.object(module.Node,'__init__',return_value=None), \
             patch.object(module.SensorNode,'declare_parameter',return_value=SimpleNamespace(value=0.)), \
             patch.object(module.SensorNode,'add_on_set_parameters_callback'), \
             patch.object(module.SensorNode,'get_logger',return_value=Mock()), \
             patch.object(module.SensorNode,'create_publisher'), \
             patch.object(module.SensorNode,'create_service'), \
             patch.object(module.SensorNode,'create_timer'), \
             patch.object(module,'select_model',return_value=MODEL), \
             patch.object(module,'eit_config_path',return_value=config), \
             patch.object(module,'python_packets',return_value=[]), \
             patch.object(module,'EITReader',Reader), \
             patch.object(module,'Inference',Engine), \
             patch.object(module.threading,'Thread',Thread), \
             patch.object(module,'parse_frame',side_effect=lambda line,count: {'frame':line}):
            sensor=module.SensorNode(args)
            sensor.stop=stop
            self.assertTrue(sensor.startup_baseline_pending)
            self.assertTrue(sensor.baseline_requested.is_set())
            sensor.receive()
        self.assertEqual(sensor.reader.opens,2)
        self.assertEqual(len(sensor.engine.captures),1)
        samples=sensor.engine.captures[0]['frames']
        self.assertEqual(len(samples),20)
        self.assertTrue(all(frame['frame'].startswith('2:') for frame in samples))
        self.assertFalse(sensor.startup_baseline_pending)
        self.assertIsNotNone(sensor.baseline_time_ns)

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
