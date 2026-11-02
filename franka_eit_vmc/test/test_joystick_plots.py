"""Axis selection must compare the commanded axis with its calibrated sensor."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('joystick_plots', ROOT / 'scripts/plot_joystick_results.py')
plots = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plots)
replay_spec = importlib.util.spec_from_file_location('joystick_replay', ROOT / 'scripts/replay_joystick.py')
replay = importlib.util.module_from_spec(replay_spec)
replay_spec.loader.exec_module(replay)


class AxisPlotTests(unittest.TestCase):
    def test_aborted_ready_trace_gets_diagnostic_video(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / 'block.json').write_text('{}')
            (folder / 'block_result.json').write_text(json.dumps(
                dict(status='error', stage='ready', error='Experiment exited')))
            snapshots = folder / 'reproducibility' / 'snapshots'
            snapshots.mkdir(parents=True)
            (snapshots / 'ready.before_motion.json').write_text(json.dumps(
                dict(status='incomplete', error='Loaded library replaced')))
            repeat = folder / 'repeat_01'
            repeat.mkdir()
            (repeat / 'joystick_ready_test.csv').write_text('host_time_ns,mode\n')
            output = folder / 'vmc_replay'
            replay.render_block(folder, output, None, fps=2)
            report = json.loads((output / 'metrics.json').read_text())
            self.assertEqual(report['status'], 'no_controller_samples')
            self.assertEqual(report['diagnostic_reason'], 'Loaded library replaced')
            self.assertEqual(report['skipped_traces'][0]['complete_samples'], 0)
            self.assertGreater((output / 'joystick_vmc_replay.mp4').stat().st_size, 0)
            self.assertGreater((output / 'joystick_vmc_summary.png').stat().st_size, 0)

    def test_failed_start_and_ready_only_runs_save_a_summary_without_esp32(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            (folder/'block_result.json').write_text(json.dumps(dict(status='error',error='USB disconnected')))
            outputs=plots.plot_block(folder)
            self.assertEqual(outputs,[folder/'run_summary.png'])
            self.assertGreater(outputs[0].stat().st_size,0)
            repeat=folder/'repeat_01';repeat.mkdir()
            (repeat/'joystick_ready_test.csv').write_text('mode,fault\nfault,ready_timeout\n')
            outputs=plots.plot_block(folder)
            self.assertEqual(outputs,[folder/'run_summary.png'])

    def test_partial_rows_are_ignored_and_interrupted_trials_are_labeled(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'partial.csv'
            path.write_text('mode,fault,retract_incomplete\ninteract,none,False\ninteract,')
            rows=plots.read_rows(path)
            self.assertEqual(len(rows),1)
            self.assertIn('INCOMPLETE',plots.trace_status(rows))
            rows[0]['mode']='done'
            self.assertEqual(plots.trace_status(rows),'completed')

    def test_preload_slip_and_approach_failure_remain_in_comparison_without_esp32(self):
        origin=1_000_000_000_000
        def row(t,mode,failed=False):
            return dict(host_time_ns=str(origin+round(t*1e9)),mode=mode,
                        trial_time_s=str(t),fault='none',theta_d_rad='0.',
                        point_m_x='.45',reference_m_x='.45',
                        trial_contact_released=str(failed),joystick_slipped=str(failed))
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);traces={}
            for condition,rows in [('baseline',[row(.01,'preload'),row(.20,'preload'),
                                                row(.29,'retract',True),row(1.,'done',True)]),
                                   ('eit',[row(-1.,'hold',True),row(.01,'preload'),row(.6,'interact'),row(18.9,'interact')])]:
                path=folder/f'joystick_{condition}_test.csv'
                with path.open('w',newline='') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
                traces[condition]=path
            esp=plots.read_esp(folder/'missing.csv')
            data=plots.pair_data(traces,esp)
            self.assertEqual(set(data),{'baseline','eit'})
            self.assertIn('slip',data['baseline'][8]['status'])
            self.assertAlmostEqual(data['baseline'][8]['event']['time_s'],.29)
            self.assertIsNone(data['eit'][8]['event'])
            self.assertEqual(data['baseline'][8]['origin'],'contact')
            self.assertAlmostEqual(data['baseline'][0][-1],.20)
            self.assertGreater(plots.comparison_limits(data)[1],19.)
            self.assertTrue(plots.draw_pair(folder,traces,esp).is_file())
            self.assertTrue(plots.draw_block(folder,[('repeat_01',data)]).is_file())
            failed=[row(.01,'approach'),row(.2,'approach'),row(.29,'fault')]
            failed[-1]['fault']='approach_timeout'
            with traces['baseline'].open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(failed[0]));writer.writeheader();writer.writerows(failed)
            only={'baseline':traces['baseline']}
            data=plots.pair_data(only,esp)
            self.assertEqual(data['baseline'][8]['origin'],'approach start')
            self.assertEqual(data['baseline'][8]['event']['reason'],'approach_timeout')
            self.assertTrue(plots.draw_pair(folder,only,esp).is_file())

    def test_contact_state_distinguishes_no_contact_from_invalid_or_stale(self):
        rows=[dict(sensor_contact='True',sensor_fresh='True',sensor_measurement_valid='True'),
              dict(sensor_contact='False',sensor_fresh='True',sensor_measurement_valid='True'),
              dict(sensor_contact='False',sensor_fresh='False',sensor_measurement_valid='True'),
              dict(sensor_contact='True',sensor_fresh='True',sensor_measurement_valid='False')]
        state=plots.contact_state(rows)
        np.testing.assert_allclose(state,[1.,0.,np.nan,np.nan],equal_nan=True)
        self.assertIsNone(plots.contact_state([dict(mode='interact')]))
        fig,axis=plots.plt.subplots()
        plots.draw_contact_lanes(axis,[('baseline',np.arange(4),state),
                                       ('eit',np.arange(4),state)])
        traces=[line for line in axis.lines if line.get_color() in plots.COLORS.values()]
        self.assertEqual(len(traces),2)
        # Equal detector readings must remain visible in separate condition lanes.
        self.assertFalse(np.allclose(traces[0].get_ydata(),traces[1].get_ydata(),equal_nan=True))
        plots.plt.close(fig)

    def test_old_traces_default_to_x(self):
        self.assertEqual(plots.selected_axis([{'theta_d_rad': '.3'}]), 'x')
        np.testing.assert_allclose(plots.command_column([{'theta_d_rad': '.3'}]), [.3])

    def test_y_trace_uses_y_reference_sensor_and_palm_tilt(self):
        calibration = json.loads((ROOT / 'config/joystick_adc_calibration.json').read_text())
        origin = 1_000_000_000_000
        rows = []
        for i in range(5):
            row = dict(host_time_ns=str(origin+i*100_000_000), mode='interact', fault='none',
                       retract_incomplete='False', joystick_axis='y', theta_d_rad='0.',
                       theta_d_y_rad=str(np.deg2rad(20.)), joystick_shaft_angle_rad='0.',
                       joystick_shaft_angle_y_rad=str(np.deg2rad(20.)), joystick_shaft_fresh='True')
            for axis in 'xyz':
                row[f'point_m_{axis}'] = str(.1 if axis == 'y' else 0.)
                row[f'reference_m_{axis}'] = str(.2 if axis == 'y' else 0.)
                row[f'requested_force_N_{axis}'] = '0.'
                row[f'external_force_N_{axis}'] = '0.'
                value = -np.sin(np.deg2rad(20.)) if axis == 'y' else -np.cos(np.deg2rad(20.)) if axis == 'z' else 0.
                row[f'pad_normal_base_{axis}'] = str(value)
                row[f'desired_pad_normal_base_{axis}'] = str(value)
            rows.append(row)
        stamps = origin + np.arange(-20, 11)*50_000_000
        # Base X stays upright; Base Y alone reaches +20 degrees.
        raw_x = calibration['mapping']['base_y_deg']['center_raw'] * (1.-20./35.)
        esp = dict(host_unix_ns=stamps, x_raw=np.where(stamps < origin, 2043., raw_x),
                   y_raw=np.full(len(stamps), 1987.))
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder/'block.json').write_text(json.dumps({'conditions':list(plots.CONDITIONS)}))
            traces = {}
            for condition in plots.CONDITIONS:
                path = folder / f'joystick_{condition}_test.csv'
                with path.open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader(); writer.writerows(rows)
                traces[condition] = path
            data = plots.pair_data(traces, esp, calibration)
            self.assertEqual(set(data),{'baseline','eit','kinematic'})
            self.assertEqual(plots.trace_condition(traces['kinematic']),'kinematic')
            self.assertEqual(plots.conditions_in_folder(folder),list(plots.CONDITIONS))
            _, point, reference, command, sensor_t, sensor_angle, orientation, axis, metadata = data['eit']
            self.assertEqual(axis, 'y')
            np.testing.assert_allclose(point, .1)
            np.testing.assert_allclose(reference, .2)
            np.testing.assert_allclose(np.rad2deg(command), 20.)
            np.testing.assert_allclose(sensor_angle[sensor_t > .1], 20.)
            for values in orientation[:3]:
                np.testing.assert_allclose(values, 20.)
            np.testing.assert_allclose(orientation[3], 0., atol=1e-5)
            outputs = [plots.draw_trial(traces['eit'], esp, calibration),
                       plots.draw_pair(folder, traces, esp, calibration),
                       plots.draw_orientation_pair(folder, data),
                       plots.draw_block(folder, [('repeat_01', data)], calibration),
                       plots.draw_run_summary(folder)]
            self.assertTrue(all(path.is_file() and path.stat().st_size > 0 for path in outputs))


if __name__ == '__main__':
    unittest.main()
