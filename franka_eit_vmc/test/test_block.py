"""Operator pacing and condition ordering, with every hardware call mocked."""
import importlib.util
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))

spec=importlib.util.spec_from_file_location('run_block',Path(__file__).resolve().parents[1]/'scripts/run_block.py')
block=importlib.util.module_from_spec(spec);spec.loader.exec_module(block)


class BlockTests(unittest.TestCase):
    def test_three_conditions_use_all_six_orders(self):
        import itertools
        conditions=['baseline','eit','kinematic']
        orders={tuple(block.condition_order(repeat,conditions)) for repeat in range(1,7)}
        self.assertEqual(orders,set(itertools.permutations(conditions)))

    def test_true_position_flag_runs_and_records_all_three_conditions(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock()
            args=['run_block.py','--task','joystick','--block','center','--repeats','1','--include-true-position','--output',d]
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',return_value=(recorder,Path(d)/'raw.csv',Path(d)/'recorder.log')),patch.object(block,'check_joystick_recorder'),patch.object(block,'wait_for_joystick_center'),patch.object(block,'wait_for_shaft_feedback') as wait,patch.object(block,'stop_controller'),patch.object(block,'stop_joystick_recorder'),patch.object(block,'plot_joystick_block'),patch.object(block.subprocess,'run') as run:
                block.main()
            wait.assert_called_once()
            trials=[call.args[0] for call in run.call_args_list if 'trial' in call.args[0]]
            self.assertEqual([args[args.index('--condition')+1] for args in trials],['baseline','eit','kinematic'])
            metadata=json.loads(next(Path(d).rglob('block.json')).read_text())
            self.assertEqual(metadata['conditions'],['baseline','eit','kinematic'])
            outcomes=[json.loads(line) for line in next(Path(d).rglob('outcomes.jsonl')).read_text().splitlines()]
            self.assertEqual([row['condition'] for row in outcomes],['baseline','eit','kinematic'])

    def setUp(self):
        archive=patch.object(block,'initialize_recording')
        archive.start();self.addCleanup(archive.stop)
        # Existing legacy tests do not spawn the calibrated-recording ROS node.
        stub=patch.object(block,'start_shaft_feedback',return_value=(None,Path('/tmp/feedback.log')))
        stub.start();self.addCleanup(stub.stop)

    def test_recording_failure_prevents_motion_for_both_tasks(self):
        for task,selection in [('joystick','center'),('ball','undisturbed')]:
            with self.subTest(task=task),tempfile.TemporaryDirectory() as folder:
                args=['run_block.py','--task',task,'--block',selection,'--output',folder]
                with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),\
                     patch.object(block,'initialize_recording',side_effect=OSError('disk full')),\
                     patch.object(block,'start_joystick_recorder') as recorder,\
                     patch.object(block,'plot_joystick_block'),patch.object(block.subprocess,'run') as run:
                    with self.assertRaisesRegex(OSError,'disk full'):block.main()
                recorder.assert_not_called();run.assert_not_called()
                saved=json.loads(next(Path(folder).rglob('block_result.json')).read_text())
                self.assertEqual(saved['stage'],'recording_conditions')

    def test_landing_offsets_come_from_block_yaml(self):
        with tempfile.TemporaryDirectory() as d:
            config=Path(d)
            path=config/'joystick_x_minus.yaml'
            path.write_text('/**:\n  ros__parameters:\n    landing_offset_x_m: -0.037\n    landing_offset_y_m: 0.004\n')
            self.assertEqual(block.configured_landing_offset('joystick','x_minus',config),
                             (path,(-.037,.004)))
            path.write_text('/**:\n  ros__parameters:\n    landing_offset_x_m: -0.052\n    landing_offset_y_m: 0.0\n')
            self.assertEqual(block.configured_landing_offset('joystick','x_minus',config)[1],
                             (-.052,0.))

    def test_two_conditions_are_paced_and_logged(self):
        with tempfile.TemporaryDirectory() as d:
            args=['run_block.py','--task','joystick','--block','center','--repeats','2','--manual','--output',d]
            recorder=Mock()
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={'landing_offset_x_m':0.}),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'joystick_recorder.log')) as start,patch.object(block,'check_joystick_recorder') as check,patch.object(block,'stop_joystick_recorder') as stop,patch.object(block,'plot_joystick_block') as plot,patch.object(block.subprocess,'run') as run,patch('builtins.input',side_effect=['r','run','ready','run','ready','run','ready','run']):
                block.main()
            start.assert_called_once()
            self.assertGreater(check.call_count,0)
            stop.assert_called_once_with(recorder)
            self.assertEqual(plot.call_count,5)
            self.assertEqual(run.call_count,8)
            trials=[c.args[0] for c in run.call_args_list if 'trial' in c.args[0]]
            self.assertEqual([x[x.index('--condition')+1] for x in trials],['baseline','eit','eit','baseline'])
            outcomes=list(Path(d).rglob('outcomes.jsonl'))
            records=[json.loads(line) for line in outcomes[0].read_text().splitlines()]
            self.assertEqual([r['repeat'] for r in records],[1,1,2,2])
            self.assertTrue(all(r['observation'] is None and
                                r['observation_status']=='pending_esp32_review' for r in records))
            self.assertRegex(outcomes[0].parent.name,
                             r'^center_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_\d{6}_[+-]\d{4}$')
            self.assertTrue(all(Path(r['joystick_recorder_csv']).parent==outcomes[0].parent
                                for r in records))

    def test_recorder_failure_prevents_robot_motion(self):
        with tempfile.TemporaryDirectory() as d:
            args=['run_block.py','--task','joystick','--block','center','--output',d]
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',side_effect=RuntimeError('USB input/output error')),patch.object(block,'plot_joystick_block') as plot,patch.object(block.subprocess,'run') as run:
                with self.assertRaisesRegex(RuntimeError,'USB input/output error'):block.main()
            run.assert_not_called()
            plot.assert_called_once()
            result=json.loads(next(Path(d).rglob('block_result.json')).read_text())
            self.assertEqual(result['status'],'error')
            self.assertIn('USB input/output error',result['error'])

    def test_ready_error_and_ctrl_c_plot_even_when_cleanup_fails(self):
        for failure in [subprocess.CalledProcessError(1,'ready'),KeyboardInterrupt()]:
            with self.subTest(failure=type(failure).__name__),tempfile.TemporaryDirectory() as d:
                recorder=Mock()
                args=['run_block.py','--task','joystick','--block','center','--output',d]
                with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',return_value=(recorder,Path(d)/'raw.csv',Path(d)/'recorder.log')),patch.object(block,'check_joystick_recorder'),patch.object(block,'wait_for_joystick_center'),patch.object(block,'stop_controller',side_effect=RuntimeError('cleanup failed')),patch.object(block,'stop_joystick_recorder'),patch.object(block,'plot_joystick_block') as plot,patch.object(block.subprocess,'run',side_effect=failure):
                    with self.assertRaises(SystemExit if isinstance(failure,KeyboardInterrupt) else subprocess.CalledProcessError):block.main()
                plot.assert_called_once()
                result=json.loads(next(Path(d).rglob('block_result.json')).read_text())
                self.assertEqual(result['stage'],'ready')
                self.assertEqual(result['status'],'interrupted' if isinstance(failure,KeyboardInterrupt) else 'error')

    def test_show_plots_opens_the_saved_summary_after_startup_error(self):
        with tempfile.TemporaryDirectory() as d:
            args=['run_block.py','--task','joystick','--block','center','--show-plots','--output',d]
            def save_summary(folder):(folder/'run_summary.png').write_bytes(b'plot')
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',side_effect=RuntimeError('USB disconnected')),patch.object(block,'plot_joystick_block',side_effect=save_summary),patch.object(block.subprocess,'Popen') as open_plot:
                with self.assertRaisesRegex(RuntimeError,'USB disconnected'):block.main()
            self.assertEqual(open_plot.call_args.args[0][0],'xdg-open')
            self.assertTrue(Path(open_plot.call_args.args[0][1]).is_file())

    def test_automatic_joystick_block_chains_both_conditions(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock()
            args=['run_block.py','--task','joystick','--block','center','--repeats','1','--output',d]
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'joystick_recorder.log')),patch.object(block,'check_joystick_recorder'),patch.object(block,'wait_for_joystick_center') as center,patch.object(block,'stop_joystick_recorder') as stop_recorder,patch.object(block,'stop_controller') as stop_controller,patch.object(block,'plot_joystick_block') as plot,patch.object(block.subprocess,'run') as run,patch('builtins.input',side_effect=AssertionError('automatic block asked for keyboard input')):
                block.main()
            self.assertEqual(run.call_count,4)
            self.assertEqual(center.call_count,4)
            self.assertEqual(plot.call_count,1)
            stop_recorder.assert_called_once_with(recorder)
            stop_controller.assert_called_once()
            folder=next(Path(d).rglob('block.json')).parent
            saved=json.loads((folder/'block.json').read_text())
            self.assertEqual((saved['center_x_raw'],saved['center_y_raw']),(2043,1987))
            self.assertTrue((folder/'joystick_adc_calibration.json').is_file())
            trials=[c.args[0] for c in run.call_args_list if 'trial' in c.args[0]]
            self.assertEqual([x[x.index('--condition')+1] for x in trials],['baseline','eit'])

    def test_automatic_joystick_requires_fresh_upright_samples(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'joystick_esp32.csv'
            stamp=time.time_ns()
            header='host_unix_ns,x_raw,y_raw\n'
            centered=''.join(f'{stamp-500_000_000+i*10_000_000},3190,3095\n' for i in range(50))
            path.write_text(header+centered)
            self.assertTrue(block.joystick_is_centered(path,3190,3095,80))
            displaced=''.join(f'{stamp-500_000_000+i*10_000_000},3190,2300\n' for i in range(50))
            path.write_text(header+displaced)
            self.assertFalse(block.joystick_is_centered(path,3190,3095,80))
            self.assertEqual(block.recent_joystick_reading(path),(3190,2300))
            stale=''.join(f'{stamp-2_500_000_000+i*10_000_000},3190,3095\n' for i in range(50))
            path.write_text(header+stale)
            self.assertFalse(block.joystick_is_centered(path,3190,3095,80))

    def test_inward_arc_accepts_stable_tilt_before_ready(self):
        recorder=Mock();recorder.poll.return_value=None
        with patch.object(block,'check_joystick_recorder'),patch.object(
                block,'recent_joystick_reading',return_value=(1139,1990)):
            block.wait_for_joystick_center(Path('/tmp/unused.csv'),recorder,
                Path('/tmp/unused.log'),2043,1987,80,allow_tilt=True)

    def test_inward_arc_accepts_stable_adc_rail(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'joystick.csv';stamp=time.time_ns()
            path.write_text('host_unix_ns,x_raw,y_raw\n'+''.join(
                f'{stamp-500_000_000+i*10_000_000},2043,4095\n' for i in range(50)))
            self.assertEqual(block.recent_joystick_reading(path),(2043,4095))
            self.assertFalse(block.joystick_is_centered(path,2043,1987,80))

    def test_automatic_inward_block_allows_tilt_for_both_conditions(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock();feedback=Mock();feedback.poll.return_value=None
            checked=dict(joystick_mechanism='inward_arc',joystick_inward_arc_m=.01,
                         joystick_length=.02,joystick_angle_deg=30.,
                         joystick_feedback_topic='/joystick/shaft')
            args=['run_block.py','--task','joystick','--block','center','--repeats','1',
                  '--mechanism','inward_arc','--output',d]
            with patch.object(sys,'argv',args),patch.object(block,'configure_joystick_mechanism'),patch.object(block,'check_controller',return_value=checked),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'joystick_recorder.log')),patch.object(block,'start_shaft_feedback',return_value=(feedback,Path(d)/'feedback.log')),patch.object(block,'wait_for_shaft_feedback'),patch.object(block,'check_joystick_recorder'),patch.object(block,'wait_for_joystick_center') as wait,patch.object(block,'stop_joystick_recorder'),patch.object(block,'stop_controller'),patch.object(block,'plot_joystick_block'),patch.object(block.subprocess,'run'),patch('builtins.input',side_effect=AssertionError('automatic block asked for input')):
                block.main()
            self.assertEqual(wait.call_count,4)
            self.assertTrue(all(call.kwargs['allow_tilt'] for call in wait.call_args_list))

    def test_quit_never_starts_motion(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(sys,'argv',['run_block.py','--task','ball','--block','push','--manual','--output',d]),patch.object(block,'check_controller',return_value={}),patch.object(block.subprocess,'run') as run,patch('builtins.input',return_value='q'):
                block.main()
            run.assert_not_called()

    def test_ball_ready_starts_without_a_prompt_but_trial_still_waits(self):
        with tempfile.TemporaryDirectory() as d:
            prompts=[]
            def respond(prompt):
                prompts.append(prompt)
                self.assertIn('run',prompt)
                return 'q'
            with patch.object(sys,'argv',['run_block.py','--task','ball','--block','undisturbed',
                                          '--repeats','1','--output',d]),                 patch.object(block,'check_controller',return_value={}),                 patch.object(block.subprocess,'run') as run,                 patch('builtins.input',side_effect=respond):
                block.main()
            self.assertEqual(run.call_count,1)
            self.assertIn('ready',run.call_args.args[0])
            self.assertEqual(len(prompts),1)

    def test_ball_outcomes_keep_camera_travel_separate_from_controller_estimate(self):
        with tempfile.TemporaryDirectory() as d:
            checked=dict(ball_axis='y',initial_ball_x=.45,initial_ball_y=-.10,
                         roll_distance=-.10,ball_duration=10.)
            def run(command,check=True):
                if 'trial' not in command:return
                condition=command[command.index('--condition')+1]
                destination=Path(command[command.index('--output')+1])
                destination.mkdir(parents=True,exist_ok=True)
                progress='-.198' if condition=='baseline' else '-.151'
                reached='True' if condition=='baseline' else 'False'
                (destination/f'ball_{condition}_test.csv').write_text(
                    'mode,fault,progress_m,pad_m_y,ball_contact_paused,ball_goal_reached\n'
                    'preload,none,-.100,-.100,False,False\n'
                    f'interact,none,{progress},-.280,False,{reached}\n'
                    f'done,none,{progress},-.280,False,{reached}\n')
            with patch.object(sys,'argv',['run_block.py','--task','ball','--block','undisturbed',
                                          '--repeats','1','--output',d]),\
                 patch.object(block,'check_controller',return_value=checked),\
                 patch.object(block.subprocess,'run',side_effect=run),\
                 patch.object(block,'render_ball_block') as render,\
                 patch('builtins.input',side_effect=['r','-8.2','slip seen','r','-6.0','more slip']):
                block.main()
            render.assert_called_once()
            records=[json.loads(line) for line in next(Path(d).rglob('outcomes.jsonl')).read_text().splitlines()]
            self.assertEqual([r['controller_completion_reason'] for r in records],
                             ['estimated_goal','time_limit'])
            for record,expected in zip(records,[-.082,-.06]):
                self.assertAlmostEqual(record['camera_ball_displacement_m'],expected)
            self.assertAlmostEqual(records[0]['camera_ball_final_axis_m'],-.182)
            self.assertAlmostEqual(records[0]['camera_ball_goal_error_m'],.018)
            self.assertNotEqual(records[0]['camera_ball_displacement_m'],
                                records[0]['controller_estimated_ball_displacement_m'])
            comparison=next(Path(d).rglob('ball_comparison.csv'))
            with comparison.open(newline='') as stream:table=list(csv.DictReader(stream))
            self.assertEqual([row['condition'] for row in table],['baseline','eit'])
            self.assertAlmostEqual(float(table[0]['camera_ball_displacement_m']),-.082)

    def test_blank_or_unknown_run_input_does_not_end_block(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(sys,'argv',['run_block.py','--task','ball','--block','undisturbed','--repeats','1','--output',d]),patch.object(block,'check_controller',return_value={}),patch.object(block.subprocess,'run') as run,patch('builtins.input',side_effect=['','unknown','r','q']):
                block.main()
            self.assertEqual(run.call_count,2)
            self.assertIn('--action',run.call_args_list[-1].args[0])
            self.assertIn('trial',run.call_args_list[-1].args[0])

    def test_failed_operation_stops_block(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(sys,'argv',['run_block.py','--task','ball','--block','undisturbed','--output',d]),patch.object(block,'check_controller',return_value={}),patch.object(block.subprocess,'run',side_effect=subprocess.CalledProcessError(1,'experiment')) as run,patch('builtins.input',return_value='ready'):
                with self.assertRaises(subprocess.CalledProcessError):block.main()
            self.assertEqual(run.call_count,1)

    def test_failed_joystick_trial_still_updates_analysis_plot(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock()
            with patch.object(sys,'argv',['run_block.py','--task','joystick','--block','center','--repeats','1','--manual','--output',d]),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'joystick_recorder.log')),patch.object(block,'check_joystick_recorder'),patch.object(block,'stop_joystick_recorder') as stop,patch.object(block,'plot_joystick_block') as plot,patch.object(block.subprocess,'run',side_effect=[None,subprocess.CalledProcessError(1,'experiment')]),patch('builtins.input',side_effect=['ready','run']):
                with self.assertRaises(subprocess.CalledProcessError):block.main()
            self.assertEqual(plot.call_count,2)
            stop.assert_called_once_with(recorder)

    def test_automatic_trial_fault_stops_before_next_condition_and_plots(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock()
            with patch.object(sys,'argv',['run_block.py','--task','joystick','--block','center','--repeats','1','--output',d]),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'joystick_recorder.log')),patch.object(block,'check_joystick_recorder'),patch.object(block,'wait_for_joystick_center'),patch.object(block,'stop_joystick_recorder') as stop_recorder,patch.object(block,'stop_controller') as stop_controller,patch.object(block,'plot_joystick_block') as plot,patch.object(block.subprocess,'run',side_effect=[None,subprocess.CalledProcessError(1,'experiment')]) as run,patch('builtins.input',side_effect=AssertionError('automatic block asked for keyboard input')):
                with self.assertRaises(subprocess.CalledProcessError):block.main()
            self.assertEqual(run.call_count,2)
            self.assertEqual(plot.call_count,1)
            stop_recorder.assert_called_once_with(recorder)
            stop_controller.assert_called_once()

    def test_retracted_contact_release_is_logged_and_next_condition_runs(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock()
            args=['run_block.py','--task','joystick','--block','center','--repeats','1','--output',d]
            results=[None,subprocess.CalledProcessError(3,'released_trial'),None,None]
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'recorder.log')),patch.object(block,'check_joystick_recorder'),patch.object(block,'wait_for_joystick_center'),patch.object(block,'stop_joystick_recorder'),patch.object(block,'stop_controller'),patch.object(block,'plot_joystick_block'),patch.object(block.subprocess,'run',side_effect=results) as run:
                block.main()
            self.assertEqual(run.call_count,4)
            records=[json.loads(line) for line in next(Path(d).rglob('outcomes.jsonl')).read_text().splitlines()]
            self.assertEqual([r['condition'] for r in records],['baseline','eit'])
            self.assertEqual(records[0]['observation_status'],'unsuccessful_contact_release')
            self.assertFalse(records[0]['observation']['success'])
            self.assertTrue(records[0]['trial_contact_released'])
            self.assertEqual(records[1]['observation_status'],'pending_esp32_review')
            self.assertFalse(records[1]['trial_contact_released'])

    def test_retracted_joystick_slip_is_failed_and_next_condition_runs(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock()
            args=['run_block.py','--task','joystick','--block','center','--repeats','1','--output',d]
            results=[None,subprocess.CalledProcessError(4,'slipped_trial'),None,None]
            with patch.object(sys,'argv',args),patch.object(block,'check_controller',return_value={}),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'recorder.log')),patch.object(block,'check_joystick_recorder'),patch.object(block,'wait_for_joystick_center'),patch.object(block,'stop_joystick_recorder'),patch.object(block,'stop_controller'),patch.object(block,'plot_joystick_block'),patch.object(block.subprocess,'run',side_effect=results) as run:
                block.main()
            self.assertEqual(run.call_count,4)
            records=[json.loads(line) for line in next(Path(d).rglob('outcomes.jsonl')).read_text().splitlines()]
            self.assertEqual([r['condition'] for r in records],['baseline','eit'])
            self.assertEqual(records[0]['observation_status'],'failed_joystick_slip')
            self.assertEqual(records[0]['observation'],{'success':False,'reason':'joystick_slip'})
            self.assertTrue(records[0]['joystick_slipped'])
            self.assertEqual(records[1]['observation_status'],'pending_esp32_review')

    def test_configuration_failure_prevents_motion(self):
        with patch.object(sys,'argv',['run_block.py','--task','joystick','--block','x_plus']),patch.object(block,'check_controller',side_effect=RuntimeError('wrong offset')),patch.object(block.subprocess,'run') as run:
            with self.assertRaisesRegex(RuntimeError,'wrong offset'):block.main()
        run.assert_not_called()

    def test_new_mechanism_is_applied_before_motion_and_starts_shared_feedback(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock();feedback=Mock();feedback.poll.return_value=None
            checked=dict(joystick_mechanism='inward_arc',joystick_inward_arc_m=.01,
                         joystick_length=.02,joystick_angle_deg=30.,joystick_feedback_topic='/joystick/shaft',
                         joystick_axis='y')
            args=['run_block.py','--task','joystick','--block','center','--repeats','1','--manual',
                  '--mechanism','inward_arc','--inward-arc-m','0.010','--axis','y','--output',d]
            with patch.object(sys,'argv',args),patch.object(block,'configure_joystick_mechanism') as configure,patch.object(block,'check_controller',return_value=checked),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'joystick_recorder.log')),patch.object(block,'start_shaft_feedback',return_value=(feedback,Path(d)/'feedback.log')) as start_feedback,patch.object(block,'wait_for_shaft_feedback') as wait,patch.object(block,'check_joystick_recorder'),patch.object(block,'stop_joystick_recorder') as stop,patch.object(block,'plot_joystick_block'),patch.object(block.subprocess,'run') as run,patch('builtins.input',side_effect=['ready','run','ready','run']):
                block.main()
            configure.assert_called_once_with('inward_arc',.01,'y')
            start_feedback.assert_called_once();wait.assert_called_once()
            self.assertEqual(run.call_count,4)
            self.assertEqual(stop.call_count,2)
            saved=json.loads(next(Path(d).rglob('block.json')).read_text())
            self.assertEqual(saved['checked_controller_parameters']['joystick_inward_arc_m'],.01)
            self.assertEqual(saved['axis'],'y')
            self.assertEqual(saved['checked_controller_parameters']['joystick_axis'],'y')

    def test_axis_alone_preserves_mechanism_and_mismatch_prevents_motion(self):
        with patch.object(sys,'argv',['run_block.py','--task','joystick','--block','center','--axis','y']),patch.object(block,'configure_joystick_mechanism') as configure,patch.object(block,'check_controller',return_value={'joystick_axis':'x'}),patch.object(block,'start_joystick_recorder') as recorder,patch.object(block.subprocess,'run') as run:
            with self.assertRaisesRegex(RuntimeError,'axis does not match'):block.main()
        configure.assert_called_once_with(None,None,'y')
        recorder.assert_not_called();run.assert_not_called()

    def test_feedback_failure_prevents_first_ready_motion(self):
        with tempfile.TemporaryDirectory() as d:
            recorder=Mock();feedback=Mock()
            checked=dict(joystick_mechanism='inward_arc',joystick_inward_arc_m=.01,
                         joystick_length=.02,joystick_angle_deg=30.,joystick_feedback_topic='/joystick/shaft')
            with patch.object(sys,'argv',['run_block.py','--task','joystick','--block','center','--manual','--output',d]),patch.object(block,'check_controller',return_value=checked),patch.object(block,'start_joystick_recorder',side_effect=lambda folder,port:(recorder,folder/'joystick_esp32.csv',folder/'recorder.log')),patch.object(block,'start_shaft_feedback',return_value=(feedback,Path(d)/'feedback.log')),patch.object(block,'wait_for_shaft_feedback',side_effect=RuntimeError('No fresh shaft')),patch.object(block,'stop_joystick_recorder'),patch.object(block,'plot_joystick_block'),patch.object(block.subprocess,'run') as run:
                with self.assertRaisesRegex(RuntimeError,'No fresh shaft'):block.main()
            run.assert_not_called()

    def test_out_of_calibration_angle_and_invalid_depth_prevent_motion(self):
        cal={'nominal_limit_deg':35.}
        cfg=dict(joystick_mechanism='inward_arc',joystick_length=.02,joystick_inward_arc_m=.01,joystick_angle_deg=45.)
        with self.assertRaisesRegex(RuntimeError,'calibrated'):block.validate_inward_configuration(cfg,cal)
        cfg.update(joystick_angle_deg=30.,joystick_inward_arc_m=.02)
        with self.assertRaisesRegex(RuntimeError,'physical joystick_length'):block.validate_inward_configuration(cfg,cal)

    def test_mechanism_flags_reject_ball_and_nonfinite_depth(self):
        for args in [['--task','ball','--block','push','--mechanism','inward_arc'],
                     ['--task','ball','--block','push','--include-true-position'],
                     ['--task','ball','--block','push','--axis','y'],
                     ['--task','joystick','--block','center','--axis','z'],
                     ['--task','joystick','--block','center','--inward-arc-m','nan']]:
            with patch.object(sys,'argv',['run_block.py']+args),patch.object(block,'configure_joystick_mechanism') as configure:
                with self.assertRaises(SystemExit) as raised:block.main()
            self.assertEqual(raised.exception.code,2);configure.assert_not_called()

if __name__=='__main__':unittest.main()
