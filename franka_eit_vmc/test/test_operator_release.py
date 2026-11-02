"""Check release results through ROS services using an isolated fake controller."""
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import json

# This test never joins the robot's ROS domain or accesses hardware.
os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 20)
os.environ['ROS_LOCALHOST_ONLY'] = '1'
source_python=str(Path(__file__).resolve().parents[1]/'python')
sys.path.insert(0,source_python)
os.environ['PYTHONPATH']=source_python+os.pathsep+os.environ.get('PYTHONPATH','')

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import SetParametersAtomically
from franka_eit_vmc.msg import ExperimentState, EitContact
from franka_eit_tools.recording import file_packet,process_identity
from rclpy.parameter import Parameter


class OperatorReleaseTests(unittest.TestCase):
    def test_combined_launch_owns_both_processes_and_closes_on_either_exit(self):
        import importlib.util
        from launch import LaunchContext
        from launch.actions import RegisterEventHandler
        from launch.event_handlers import OnProcessExit
        from launch_ros.actions import Node as LaunchNode
        path=Path(__file__).resolve().parents[1]/'launch/controller.launch.py'
        spec=importlib.util.spec_from_file_location('franka_controller_launch_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        config=Path(__file__).resolve().parents[1]/'config'
        context=LaunchContext()
        context.launch_configurations.update(task='ball',config=str(config/'controller.yaml'),
            mount='',posture='',overrides=str(config/'ball_y_minus_10cm.yaml'),
            with_eit='true',eit_model='',eit_config='')
        entities=module.setup(context)
        nodes=[entry for entry in entities if isinstance(entry,LaunchNode)]
        handlers=[entry for entry in entities if isinstance(entry,RegisterEventHandler)]
        self.assertEqual(len(nodes),2)
        self.assertEqual(len(handlers),2)
        self.assertTrue(all(isinstance(entry.event_handler,OnProcessExit) for entry in handlers))
        self.assertEqual({entry.event_handler._OnActionEventBase__action_matcher for entry in handlers},set(nodes))
        context.launch_configurations['with_eit']='false'
        single=module.setup(context)
        self.assertEqual(len(single),1)
        self.assertIsInstance(single[0],LaunchNode)

    def test_axis_yaml_values_are_strings_in_the_native_ros_parser(self):
        from rclpy.context import Context
        config=Path(__file__).resolve().parents[1]/'config'
        for files,name,expected in [(['controller.yaml','ball.yaml','ball_y_minus_10cm.yaml'],'ball_axis','y'),
                                    (['controller.yaml','joystick.yaml'],'joystick_axis','x')]:
            with self.subTest(parameter=name):
                context=Context();args=['--ros-args']
                for filename in files:args.extend(['--params-file',str(config/filename)])
                rclpy.init(args=args,context=context)
                node=Node('axis_yaml_test',context=context,enable_rosout=False,start_parameter_services=False)
                try:self.assertEqual(node.declare_parameter(name,'x').value,expected)
                finally:node.destroy_node();rclpy.shutdown(context=context)

    def test_axis_configuration_waits_for_realtime_status(self):
        rclpy.init()
        node = Node('axis_test_fake_controller')
        publisher = node.create_publisher(ExperimentState, '/eit_joystick/status', 10)
        state = dict(mechanism='legacy', depth=.01, axis='x', pending=None, applied_at=None)
        requests = []

        def configure(request, response):
            values = {p.name: (p.value.string_value if p.value.type == 4 else p.value.double_value)
                      for p in request.parameters}
            requests.append(values)
            state['pending'] = (time.monotonic()+.25, values)
            response.result.successful = True
            return response

        def publish():
            if state['pending'] and time.monotonic() >= state['pending'][0]:
                values = state['pending'][1]
                for parameter, key in [('joystick_mechanism', 'mechanism'),
                                       ('joystick_inward_arc_m', 'depth'), ('joystick_axis', 'axis')]:
                    if parameter in values: state[key] = values[parameter]
                state['pending'] = None
                state['applied_at'] = time.monotonic()
            message = ExperimentState()
            message.mode_name = 'hold'
            message.joystick_mechanism = state['mechanism']
            message.joystick_inward_arc_m = state['depth']
            message.joystick_axis = state['axis']
            publisher.publish(message)

        service = node.create_service(SetParametersAtomically, '/eit_joystick/set_parameters_atomically', configure)
        timer = node.create_timer(.02, publish)
        executor = SingleThreadedExecutor();executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True);thread.start()
        try:
            scripts = Path(__file__).resolve().parents[1] / 'scripts'
            for selection, expected in [("'inward_arc', .005, 'y'", dict(
                    joystick_mechanism='inward_arc', joystick_inward_arc_m=.005, joystick_axis='y')),
                    ("None, None, 'x'", dict(joystick_axis='x'))]:
                state['applied_at'] = None
                code = (f'import sys;sys.path.insert(0,{str(scripts)!r});'
                        f'import run_block;run_block.configure_joystick_mechanism({selection})')
                result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=15.)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertEqual(requests[-1], expected)
                self.assertIsNotNone(state['applied_at'], 'helper returned before status acknowledgment')
            self.assertEqual((state['mechanism'], state['depth'], state['axis']), ('inward_arc', .005, 'x'))
        finally:
            timer.cancel();executor.shutdown(timeout_sec=2.);thread.join(timeout=2.)
            node.destroy_node();rclpy.shutdown()

    def test_only_a_completed_release_retraction_returns_the_continue_result(self):
        self.check_operator_recording('joystick')

    def test_ball_trials_preserve_all_controller_and_sensor_conditions(self):
        self.check_operator_recording('ball')

    def check_operator_recording(self,task):
        rclpy.init()
        target='/eit_joystick' if task=='joystick' else '/eit_ball_roll'
        node = Node(target.lstrip('/'))
        packet=file_packet(__file__,'loaded_eit_model')
        build=dict(schema_version=1,source_archive=dict(path='test_source.tar.gz',
                   sha256=packet['sha256'],data_base64=packet['data_base64']))
        node.declare_parameter('recording_process_id',os.getpid())
        identity=process_identity()
        node.declare_parameter('recording_process_boot_id',identity['process_boot_id'])
        node.declare_parameter('recording_process_start_ticks',identity['process_start_ticks'])
        node.declare_parameter('recording_build_manifest',json.dumps(build))
        node.declare_parameter('joystick_mechanism','legacy')
        node.declare_parameter('preload_time',.678)
        node.declare_parameter('ball_duration',8.456)
        sensor_node=Node('eit_contact')
        sensor_node.declare_parameter('contact_threshold',.123)
        feedback_node=Node('eit_joystick_shaft_feedback')
        sensor_state={'baseline':'preserved baseline','calibrations':0}
        def sensor_record(request,response):
            response.success=True
            response.message=json.dumps(dict(**process_identity(),model_sha256=packet['sha256'],
                baseline={'frames':[{'raw_line':sensor_state['baseline']}]},failed=None,artifacts=[packet]))
            return response
        def calibrate(request,response):
            sensor_state['baseline']='baseline after run'
            sensor_state['calibrations']+=1
            response.success=True
            response.message='Baseline calibrated from 20 fresh electrical frames; pad must be unloaded'
            return response
        def feedback_record(request,response):
            response.success=True
            calibration=packet|{'role':'loaded_joystick_calibration'}
            response.message=json.dumps(dict(**process_identity(),decoder={'filter_s':.05},artifacts=[calibration]))
            return response
        recording_services=[sensor_node.create_service(Trigger,'~/recording_state',sensor_record),
                            feedback_node.create_service(Trigger,'~/recording_state',feedback_record),
                            sensor_node.create_service(Trigger,'~/calibrate_baseline',calibrate)]
        publisher = node.create_publisher(ExperimentState, target+'/status', 10)
        sensor_publisher=sensor_node.create_publisher(EitContact,'/eit/contact',10)
        state = {'started': None, 'released': True, 'slipped': False, 'incomplete': False, 'stops': 0, 'source':'eit',
                 'start_calibrations':None}

        def start(request, response):
            state['start_calibrations']=sensor_state['calibrations']
            # Before/after dumps alone would miss this temporary threshold change.
            sensor_node.set_parameters([Parameter('contact_threshold',value=.456)])
            sensor_node.set_parameters([Parameter('contact_threshold',value=.123)])
            state['started'] = time.monotonic()
            response.success = True
            return response

        def stop(request, response):
            state['started'] = None
            state['stops'] += 1
            response.success = True
            return response

        def condition(request, response):
            state['source']='eit' if request.data else 'baseline'
            response.success = True
            return response

        def kinematic(request, response):
            state['source']='kinematic';response.success=True;return response

        def publish():
            sensor_message=EitContact()
            stamp=time.time_ns()
            sensor_message.header.stamp.sec=stamp//1_000_000_000
            sensor_message.header.stamp.nanosec=stamp%1_000_000_000
            sensor_message.model_is_physical=True
            sensor_message.measurement_valid=True
            sensor_publisher.publish(sensor_message)
            message = ExperimentState()
            elapsed = 0. if state['started'] is None else time.monotonic() - state['started']
            message.mode_name = ('hold' if state['started'] is None else
                                 'interact' if elapsed < .25 else
                                 'retract' if elapsed < .50 else 'done')
            message.fault_name = 'none'
            message.contact_source = state['source']
            message.use_eit = state['source']=='eit'
            message.trial_contact_released = state['released'] and elapsed >= .25
            message.joystick_slipped = state['slipped'] and elapsed >= .25
            message.retract_incomplete = state['incomplete'] and message.mode_name == 'done'
            publisher.publish(message)

        services = [node.create_service(Trigger, target+'/start_trial', start),
                    node.create_service(Trigger, target+'/stop', stop),
                    node.create_service(SetBool, target+'/use_eit', condition),
                    node.create_service(Trigger, target+'/use_kinematic_position', kinematic)]
        timer = node.create_timer(.02, publish)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(sensor_node);executor.add_node(feedback_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            script = Path(__file__).resolve().parents[1] / 'scripts' / 'experiment.py'
            cases=([(True, False, False, 3,'baseline'), (True, False, True, 4,'eit'),
                    (True, True, False, 1,'eit'), (False, False, False, 0,'eit'),
                    (False,False,False,0,'kinematic')] if task=='joystick'
                   else [(False,False,False,0,'baseline'),(False,False,False,0,'eit')])
            for released, incomplete, slipped, expected, source in cases:
                with self.subTest(released=released, incomplete=incomplete), tempfile.TemporaryDirectory() as folder:
                    state.update(started=None, released=released, slipped=slipped, incomplete=incomplete, stops=0)
                    sensor_state['baseline']='preserved baseline'
                    calibrations_before=sensor_state['calibrations']
                    operator_name='recording_operator_'+task+'_'+str(time.monotonic_ns())
                    result = subprocess.run([sys.executable, str(script), '--task', task,
                                             '--action', 'trial', '--condition',source,'--output', folder,
                                             '--ros-args','-r','__node:='+operator_name],
                                            capture_output=True, text=True, timeout=15.)
                    self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
                    snapshots=list((Path(folder)/'reproducibility'/'snapshots').glob('*.json'))
                    before=json.loads(next(p for p in snapshots if p.name.endswith('.before_motion.json')).read_text())
                    self.assertEqual(before['status'],'complete')
                    self.assertEqual(before['context']['condition'],source)
                    self.assertEqual(before['nodes'][target]['parameters']['preload_time']['value'],.678)
                    self.assertEqual(before['nodes'][target]['parameters']['ball_duration']['value'],8.456)
                    self.assertEqual(before['nodes']['/eit_contact']['runtime']['baseline']['frames'][0]['raw_line'],
                                     'baseline after run' if task=='ball' else 'preserved baseline')
                    self.assertEqual(sensor_state['calibrations']-calibrations_before,1 if task=='ball' else 0)
                    self.assertEqual(state['start_calibrations'],sensor_state['calibrations'])
                    events=[json.loads(line)['event'] for line in
                            (Path(folder)/'reproducibility/parameter_events.jsonl').read_text().splitlines()]
                    changes=[p['value']['double_value'] for event in events
                             for p in event['changed_parameters'] if p['name']=='contact_threshold']
                    self.assertIn(.456,changes)
                    self.assertIn(.123,changes)
                    with next(Path(folder).glob('*.csv')).open() as stream:
                        rows = list(csv.DictReader(stream))
                    self.assertEqual(rows[-1]['trial_contact_released'], str(released))
                    self.assertEqual(rows[-1]['joystick_slipped'], str(slipped))
                    self.assertEqual(rows[-1]['contact_source'],source)
                    if incomplete:
                        self.assertGreater(state['stops'], 0)
                        self.assertIn('could not finish retraction', result.stderr)
                    else:
                        self.assertEqual(state['stops'], 0)
        finally:
            timer.cancel()
            executor.shutdown(timeout_sec=2.)
            thread.join(timeout=2.)
            node.destroy_node()
            sensor_node.destroy_node();feedback_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
