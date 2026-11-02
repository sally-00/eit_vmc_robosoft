#!/usr/bin/env python3
"""Run paired physical trials; ball ready moves start when the block starts."""
import argparse
import csv
import json
import math
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
import yaml
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from franka_eit_tools.recording import initialize_recording,record_command


def joystick_recorder_script():
    for parent in Path(__file__).resolve().parents:
        candidate=parent/'joystick_hardware'/'run_joystick.sh'
        if candidate.is_file():return candidate
    raise RuntimeError('Cannot find joystick_hardware/run_joystick.sh beside this checkout')


def stop_joystick_recorder(process):
    if process.poll() is not None:return
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=5.)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:process.wait(timeout=2.)
        except subprocess.TimeoutExpired:process.kill();process.wait()


def check_joystick_recorder(process,log_path):
    code=process.poll()
    if code is not None:
        lines=log_path.read_text(errors='replace').splitlines() if log_path.exists() else []
        detail=lines[-1] if lines else f'exit code {code}'
        raise RuntimeError(f'ESP32 recorder stopped: {detail} (see {log_path})')


def start_joystick_recorder(folder,port):
    csv_path=folder/'joystick_esp32.csv'
    log_path=folder/'joystick_recorder.log'
    command=[str(joystick_recorder_script()),'--no-plot','--output',str(csv_path),
             '--parent-pid',str(os.getpid())]
    if port:command.extend(['--port',port])
    record_command(folder,'esp32_recorder',command)
    with log_path.open('x') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                 start_new_session=True)
    try:
        deadline=time.monotonic()+10.
        while time.monotonic()<deadline:
            check_joystick_recorder(process,log_path)
            if csv_path.exists() and len(csv_path.read_text(errors='replace').splitlines())>1:
                return process,csv_path,log_path
            time.sleep(.1)
        raise RuntimeError(f'ESP32 recorder opened but no samples arrived in 10 s (see {log_path})')
    except BaseException:
        stop_joystick_recorder(process)
        raise


def recent_joystick_reading(path):
    with path.open(newline='') as stream:
        rows=deque(csv.DictReader(stream),maxlen=50)
    samples=[]
    for row in rows:
        try:samples.append((int(row['host_unix_ns']),int(row['x_raw']),int(row['y_raw'])))
        except (KeyError,TypeError,ValueError):continue
    if len(samples)<30 or time.time_ns()-samples[-1][0]>500_000_000:return None
    xs=[row[1] for row in samples];ys=[row[2] for row in samples]
    x=statistics.median(xs);y=statistics.median(ys)
    if (not 0<=x<=4095 or not 0<=y<=4095 or
            max(abs(value-x) for value in xs)>60 or
            max(abs(value-y) for value in ys)>60):return None
    return x,y


def joystick_is_centered(path,center_x,center_y,tolerance):
    reading=recent_joystick_reading(path)
    return (reading is not None and abs(reading[0]-center_x)<=tolerance and
            abs(reading[1]-center_y)<=tolerance)


def wait_for_joystick_center(csv_path,recorder,recorder_log,center_x,center_y,tolerance,
                             allow_tilt=False):
    deadline=time.monotonic()+20.
    waiting=False
    while True:
        check_joystick_recorder(recorder,recorder_log)
        reading=recent_joystick_reading(csv_path)
        if reading is not None and (allow_tilt or
                (abs(reading[0]-center_x)<=tolerance and
                 abs(reading[1]-center_y)<=tolerance)):
            if allow_tilt:
                print(f'Stable ESP32 start reading: X={reading[0]:.0f}, Y={reading[1]:.0f} raw; '
                      'inward_arc uses calibrated absolute angles.',flush=True)
            return
        if time.monotonic()>=deadline:
            requirement='stable reading' if allow_tilt else 'calibrated centre'
            raise RuntimeError(f'Joystick did not reach a {requirement} within 20 s; '
                               f'check the fixture and ESP32 recording {csv_path}')
        if not waiting:
            print('Waiting for stable joystick feedback before automatic motion...' if allow_tilt else
                  'Waiting for the joystick to return upright before automatic motion...',flush=True)
            waiting=True
        time.sleep(.1)


def stop_controller(script,task):
    try:
        result=subprocess.run([sys.executable,str(script),'--task',task,'--action','stop'],
                              capture_output=True,text=True,timeout=12.)
        if result.returncode==0:print('Controller stop requested after automatic block.',flush=True)
    except (OSError,subprocess.TimeoutExpired) as exc:
        print('Could not confirm controller stop: '+str(exc),file=sys.stderr,flush=True)


def plot_joystick_block(folder):
    script=Path(__file__).with_name('plot_joystick_results.py')
    try:
        result=subprocess.run([sys.executable,str(script),str(folder)],
                              capture_output=True,text=True,timeout=60.,start_new_session=True)
    except (OSError,subprocess.TimeoutExpired) as exc:
        print('Plot generation failed; trial CSVs are saved: '+str(exc),file=sys.stderr,flush=True)
        return
    if result.returncode:
        print('Plot generation failed; trial CSVs are saved: '+
              (result.stderr.strip().splitlines()[-1] if result.stderr.strip() else
               f'exit code {result.returncode}'),file=sys.stderr,flush=True)
    elif result.stdout.strip():
        print('Analysis/comparison plots updated in:',folder,flush=True)


def require_command(prompt,valid):
    while True:
        try:
            answer=input('INPUT REQUIRED: '+prompt).strip().lower()
        except EOFError:
            print('\nInput stream closed; block stopped before the next motion.',flush=True)
            return None
        if answer in valid:return answer
        if answer=='q':
            print('Block stopped by operator before the next motion.',flush=True)
            return None
        print('No motion started. Type '+', '.join(valid)+' or q, then press Enter.',flush=True)


def config_directory(override=None):
    if override:return Path(override).expanduser()
    if os.environ.get('EIT_CONFIG_DIR'):return Path(os.environ['EIT_CONFIG_DIR']).expanduser()
    for parent in Path(__file__).resolve().parents:
        candidate=parent/'franka_eit_vmc'/'config'
        if candidate.is_dir():return candidate
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('franka_eit_vmc'))/'config'


def configured_landing_offset(task,block,config_dir):
    path=config_dir/(f'joystick_{block}.yaml' if task=='joystick' else 'controller.yaml')
    try:
        params=yaml.safe_load(path.read_text())['/**']['ros__parameters']
        offsets=(float(params['landing_offset_x_m']),float(params['landing_offset_y_m']))
    except (OSError,KeyError,TypeError,ValueError,yaml.YAMLError) as exc:
        raise RuntimeError(f'Cannot read landing offsets from {path}: {exc}') from exc
    if not all(math.isfinite(value) for value in offsets):
        raise RuntimeError(f'Landing offsets must be finite in {path}')
    return path,offsets


def check_controller(task,block,config_dir=None):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    rclpy.init()
    node=rclpy.create_node('eit_block_config_check')
    try:
        name='/eit_joystick' if task=='joystick' else '/eit_ball_roll'
        client=node.create_client(GetParameters,name+'/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Launch the task controller first')
        names=['landing_offset_x_m','landing_offset_y_m','update_ball_attachment','tool_weight_compensation_mass_kg']
        if task=='joystick':names += ['joystick_mechanism','joystick_axis','joystick_inward_arc_m','joystick_length',
            'joystick_angle_deg','joystick_pivot_m','joystick_alignment_enabled',
            'joystick_duration','joystick_return_zero_start_s',
            'joystick_approach_final_hold_s',
            'joystick_task_spring_uses_shaft','joystick_alignment_half_span_m',
            'joystick_update_eit_attachment','joystick_attachment_filter_tau_s',
            'joystick_slip_contact_mismatch_m',
            'joystick_slip_confirm_s',
            'joystick_slider_radius_m','joystick_alignment_spring_k','joystick_alignment_spring_d',
            'joystick_feedback_topic','joystick_feedback_timeout',
            'joystick_release_drop_deg','joystick_release_upright_deg',
            'joystick_release_min_command_deg','joystick_release_window_s',
            'joystick_task_spring_k','joystick_task_spring_d',
            'joystick_task_spring_force_limit',
            'joystick_alignment_spring_force_limit']
        if task=='ball':names += ['ball_axis','initial_ball_x','initial_ball_y','roll_distance',
            'ball_baseline_motion_ratio','ball_interact_dropout_grace_s',
            'ball_reference_speed_m_s',
            'ball_reference_advance_speed_m_s','ball_reference_advance_limit_m',
            'ball_reference_advance_start_error_m',
            'ball_duration','ball_goal_tolerance_m','ball_goal_dwell_s',
            'ball_recover_contact','ball_initial_attachment','ball_goal_d','ball_force_slew',
            'ball_approach_force_limit',
            'ball_normal_mass','ball_normal_damping','ball_normal_speed','ball_normal_travel',
            'ball_normal_up_travel','ball_goal_ramp_s','ball_pause_k','ball_pause_d',
            'ball_position_filter','ball_velocity_filter','progress_speed','ball_contact_settle_s','ball_contact_loss_grace_s','ready_pad_position_m','workspace_min_m','workspace_max_m']
        future=client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        if not future.done() or future.result() is None:raise RuntimeError('Cannot read task configuration')
        values=future.result().values
        if any(v.type==0 for v in values):raise RuntimeError('Controller configuration is incomplete')
        x,y=values[0].double_value,values[1].double_value
        path,(expected_x,expected_y)=configured_landing_offset(
            task,block,config_directory(config_dir))
        if not all(math.isfinite(value) for value in (x,y)) or not all(
                math.isclose(actual,expected,rel_tol=0.,abs_tol=1e-6)
                for actual,expected in ((x,expected_x),(y,expected_y))):
            raise RuntimeError(f'Controller landing offsets differ from {path}: '
                               f'active X={x:+.6f} m, Y={y:+.6f} m; '
                               f'configured X={expected_x:+.6f} m, Y={expected_y:+.6f} m')
        if task=='ball' and values[2].bool_value:raise RuntimeError('Ball comparison requires update_ball_attachment=false')
        checked={key:parameter_value(v) for key,v in zip(names,values)}
        if task=='ball':
            config_dir=config_directory(config_dir)
            expected=yaml.safe_load((config_dir/'ball.yaml').read_text())['/**']['ros__parameters']
            if checked['ball_axis']=='y':
                overlay=config_dir/'ball_y_minus_10cm.yaml'
                expected=expected|yaml.safe_load(overlay.read_text())['/**']['ros__parameters']
            for key in ('initial_ball_y','ready_pad_position_m','ball_reference_advance_limit_m'):
                configured=expected[key]
                active=checked[key]
                match=(len(active)==len(configured) and all(
                    math.isclose(x,y,rel_tol=0.,abs_tol=1e-6)
                    for x,y in zip(active,configured))) if isinstance(active,list) else math.isclose(
                        active,configured,rel_tol=0.,abs_tol=1e-6)
                if not match:
                    raise RuntimeError(f'Ball controller has stale {key}: active {active}, '
                                       f'configured {configured}; restart the ball launch before running a block')
        if task=='ball' and (not checked['ball_recover_contact'] or checked['ball_initial_attachment']):
            raise RuntimeError('Position-feedback block requires ball_recover_contact=true and ball_initial_attachment=false')
        return checked
    finally:
        node.destroy_node();rclpy.shutdown()


def parameter_value(value):
    fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',8:'double_array_value'}
    if value.type not in fields:raise RuntimeError('Unsupported/incomplete controller parameter')
    result=getattr(value,fields[value.type])
    return list(result) if value.type==8 else result


def ball_trial_summary(destination,condition,axis,initial):
    traces=sorted(destination.glob(f'ball_{condition}_*.csv'))
    if not traces:return dict(controller_completion_reason='no_trace')
    with traces[-1].open(newline='') as stream:rows=list(csv.DictReader(stream))
    if not rows:return dict(controller_completion_reason='empty_trace',trace=str(traces[-1]))
    engaged=next((row for row in rows if row['mode'] in ('preload','interact')),None)
    last_motion=next((row for row in reversed(rows) if row['mode']=='interact'),None)
    final=rows[-1]
    reached=final.get('ball_goal_reached')=='True'
    reason='estimated_goal' if reached else 'time_limit' if final['mode']=='done' else 'fault_or_incomplete'
    return dict(trace=str(traces[-1]),controller_completion_reason=reason,
                controller_goal_reached=reached,
                controller_estimated_ball_final_axis_m=float(final['progress_m']) if engaged else None,
                controller_estimated_ball_displacement_m=float(final['progress_m'])-initial if engaged else None,
                robot_pad_displacement_m=(float(last_motion[f'pad_m_{axis}'])-
                    float(engaged[f'pad_m_{axis}'])) if engaged and last_motion else None,
                rolling_paused_fraction=(sum(row['ball_contact_paused']=='True' for row in rows
                    if row['mode']=='interact')/sum(row['mode']=='interact' for row in rows))
                    if last_motion else None)


def camera_ball_displacement(axis):
    while True:
        raw=input(f'Camera-measured ball displacement along Base {axis.upper()} '
                  '(cm, signed; e.g. -8.2 for -Y; pending if not measured yet; q stops): ').strip()
        if raw.lower()=='q':return 'quit',None
        if not raw or raw.lower()=='pending':return 'pending_camera_measurement',None
        try:value=float(raw)
        except ValueError:value=float('nan')
        if math.isfinite(value) and abs(value)<=100.:
            return 'observed',value/100.
        print('Enter a finite signed displacement in cm, or pending.',flush=True)


def render_ball_block(folder):
    if not any(folder.glob('repeat_*/ball_baseline_*.csv')) and not any(folder.glob('repeat_*/ball_eit_*.csv')):
        return
    script=Path(__file__).with_name('replay_ball.py')
    output=folder/'vmc_replay'
    try:
        subprocess.run([sys.executable,str(script),str(folder),'--output',str(output),'--all'],check=True)
        print('Ball VMC videos:',output,flush=True)
    except Exception as exc:
        print('Ball video generation failed; trial CSVs are saved: '+str(exc),file=sys.stderr,flush=True)


def render_joystick_block(folder):
    if not any(folder.glob('repeat_*/joystick_*.csv')):
        return
    script=Path(__file__).with_name('replay_joystick.py')
    output=folder/'vmc_replay'
    try:
        subprocess.run([sys.executable,str(script),str(folder),'--output',str(output),
                        '--fps','8','--speed','2'],check=True)
        print('Joystick VMC videos:',output,flush=True)
    except Exception as exc:
        print('Joystick video generation failed; trial CSVs are saved: '+str(exc),
              file=sys.stderr,flush=True)


def write_ball_comparison(folder):
    outcomes=folder/'outcomes.jsonl'
    if not outcomes.is_file():return
    records=[json.loads(line) for line in outcomes.read_text().splitlines() if line.strip()]
    records=[row for row in records if row.get('condition') in ('baseline','eit')]
    if not records:return
    fields=['repeat','condition','observation_status','controller_completion_reason',
            'controller_goal_reached','camera_ball_start_axis_m',
            'camera_ball_displacement_m','camera_ball_final_axis_m',
            'camera_ball_relative_goal_axis_m','camera_ball_goal_error_m',
            'camera_ball_absolute_goal_error_m',
            'controller_estimated_ball_displacement_m','robot_pad_displacement_m','trace']
    with (folder/'ball_comparison.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader()
        for row in records:writer.writerow({key:row.get(key) for key in fields})


def condition_order(repeat,conditions):
    if len(conditions)==2:return conditions if repeat%2 else list(reversed(conditions))
    offset=(repeat-1)%len(conditions)
    order=conditions[offset:]+conditions[:offset]
    return list(reversed(order)) if ((repeat-1)//len(conditions))%2 else order


def configure_joystick_mechanism(mechanism,depth,axis=None):
    """Change only while holding; wait for actual realtime-core acknowledgment."""
    import rclpy
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically
    from franka_eit_vmc.msg import ExperimentState
    rclpy.init();node=rclpy.create_node('eit_block_mechanism_config')
    latest=[None]
    sub=node.create_subscription(ExperimentState,'/eit_joystick/status',lambda m:latest.__setitem__(0,m),10)
    try:
        deadline=time.monotonic()+5.
        while latest[0] is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if latest[0] is None or latest[0].mode_name not in ('hold','done'):
            raise RuntimeError('Launch the joystick controller and leave it in hold/done before changing mechanism/axis')
        client=node.create_client(SetParametersAtomically,'/eit_joystick/set_parameters_atomically')
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Controller parameter service unavailable')
        parameters=[]
        if mechanism is not None:parameters.append(Parameter('joystick_mechanism',value=mechanism).to_parameter_msg())
        if depth is not None:parameters.append(Parameter('joystick_inward_arc_m',value=depth).to_parameter_msg())
        if axis is not None:parameters.append(Parameter('joystick_axis',value=axis).to_parameter_msg())
        expected_mechanism=mechanism if mechanism is not None else latest[0].joystick_mechanism
        expected_depth=depth if depth is not None else latest[0].joystick_inward_arc_m
        expected_axis=axis if axis is not None else latest[0].joystick_axis
        future=client.call_async(SetParametersAtomically.Request(parameters=parameters))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        if not future.done() or future.result() is None:raise RuntimeError('Mechanism update timed out')
        if not future.result().result.successful:raise RuntimeError(future.result().result.reason)
        latest[0]=None;deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05);m=latest[0]
            if m and m.mode_name in ('hold','done') and m.joystick_mechanism==expected_mechanism and (
                    m.joystick_axis==expected_axis and
                    math.isclose(m.joystick_inward_arc_m,expected_depth,abs_tol=1e-9)):
                return
        raise RuntimeError('Realtime controller did not acknowledge the selected mechanism/depth/axis')
    finally:node.destroy_node();rclpy.shutdown()


def validate_inward_configuration(checked,calibration):
    if checked.get('joystick_mechanism')!='inward_arc':return
    depth=checked['joystick_inward_arc_m'];length=checked['joystick_length']
    if not all(math.isfinite(x) for x in (depth,length)) or not 0<=depth<length:
        raise RuntimeError('Inward arc must be nonnegative and below physical joystick_length')
    limit=float(calibration['nominal_limit_deg']);angle=checked['joystick_angle_deg']
    if not math.isfinite(limit) or not math.isfinite(angle) or not 0<angle<limit:
        raise RuntimeError(f'Inward-arc live feedback is calibrated to ±{limit:g} deg; '
                           f'active joystick_angle_deg={angle:g}. Set joystick_angle_deg below '
                           f'{limit:g} in joystick.yaml beside the launch controller.yaml, '
                           'then restart the controller.')


def start_shaft_feedback(folder,csv_path,topic,center_x=None,center_y=None):
    script=Path(__file__).with_name('joystick_feedback_node.py')
    log_path=folder/'joystick_feedback.log'
    command=[sys.executable,str(script),'--csv',str(csv_path),
        '--calibration',str(folder/'joystick_adc_calibration.json'),'--topic',topic,
        '--output',str(folder/'joystick_calibrated.csv'),
        '--parent-pid',str(os.getpid())]
    if center_x is not None:command.extend(['--center-x-raw',str(center_x)])
    if center_y is not None:command.extend(['--center-y-raw',str(center_y)])
    record_command(folder,'shaft_feedback',command)
    with log_path.open('x') as log:
        process=subprocess.Popen(command,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    return process,log_path


def wait_for_shaft_feedback(process,log_path):
    import rclpy
    from franka_eit_vmc.msg import ExperimentState
    rclpy.init();node=rclpy.create_node('eit_block_shaft_check');latest=[None]
    sub=node.create_subscription(ExperimentState,'/eit_joystick/status',lambda m:latest.__setitem__(0,m),10)
    try:
        deadline=time.monotonic()+8.
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError(f'Shaft feedback process stopped; see {log_path}')
            rclpy.spin_once(node,timeout_sec=.05);m=latest[0]
            if m and m.joystick_shaft_fresh:return
        raise RuntimeError(f'No fresh calibrated shaft feedback; see {log_path}')
    finally:node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=['joystick','ball'],required=True)
    p.add_argument('--block',required=True,help='Joystick: center, x_minus, x_plus, y_minus, y_plus. Ball: undisturbed, push.')
    p.add_argument('--repeats',type=int,default=5)
    p.add_argument('--mechanism',choices=['legacy','inward_arc'],help='Joystick only: select mechanism between trials; omitted keeps controller configuration')
    p.add_argument('--axis',choices=['x','y'],help='Joystick Base tilt direction: x=current (rotation about +Y), y=other axis (rotation about -X); omitted keeps controller configuration')
    p.add_argument('--inward-arc-m',type=float,help='Joystick inward_arc only: inward shortening of BOTH arcs in metres, e.g. 0.010; default comes from controller.yaml')
    p.add_argument('--output',type=Path,default=Path.home()/'eit_trials')
    p.add_argument('--config-dir',type=Path,help='Directory containing the same YAML files used to launch the controller; defaults to EIT_CONFIG_DIR or this checkout')
    p.add_argument('--conditions-file',type=Path,help='Optional physical setup notes / measurements / camera settings to preserve with the run')
    p.add_argument('--joystick-port',help='ESP32 serial port; defaults to CH340 auto-detection')
    p.add_argument('--manual',action='store_true',help='Keep ready/run prompts for joystick blocks and the ready prompt for ball blocks')
    p.add_argument('--show-plots',action='store_true',help='Open the saved joystick run_summary.png on the desktop after exit, including errors')
    p.add_argument('--include-true-position',action='store_true',help='Joystick only: add a third kinematic tip-position condition using measured shaft angles, pivot and length')
    p.add_argument('--center-x-raw',type=int,help='Override the calibrated upright ESP32 X reading')
    p.add_argument('--center-y-raw',type=int,help='Override the calibrated upright ESP32 Y reading')
    p.add_argument('--center-tolerance-raw',type=int,default=80,help='Allowed ADC difference from upright on each axis')
    a=p.parse_args()
    if a.task!='joystick' and (a.mechanism is not None or a.inward_arc_m is not None or a.axis is not None):p.error('Mechanism/axis options apply only to joystick blocks')
    if a.task!='joystick' and a.include_true_position:p.error('--include-true-position applies only to joystick blocks')
    if a.inward_arc_m is not None:
        if not math.isfinite(a.inward_arc_m) or a.inward_arc_m<0:p.error('--inward-arc-m must be finite and nonnegative')
        if a.mechanism=='legacy':p.error('--inward-arc-m is only used by --mechanism inward_arc')
        if a.mechanism is None:a.mechanism='inward_arc'
    calibration_path=config_directory(a.config_dir)/'joystick_adc_calibration.json'
    calibration=None
    if a.task=='joystick':
        if not calibration_path.is_file():
            raise RuntimeError(f'Missing joystick ADC calibration: {calibration_path}')
        calibration=json.loads(calibration_path.read_text())
        center=calibration['endpoints']['center']
        if a.center_x_raw is None:a.center_x_raw=round(center['x_raw'])
        if a.center_y_raw is None:a.center_y_raw=round(center['y_raw'])
    if a.center_x_raw is None:a.center_x_raw=0
    if a.center_y_raw is None:a.center_y_raw=0
    blocks=['center','x_minus','x_plus','y_minus','y_plus'] if a.task=='joystick' else ['undisturbed','push']
    if a.block not in blocks or not 1<=a.repeats<=100:p.error('Choose a valid block and 1–100 repeats')
    if a.center_tolerance_raw<=0 or not all(0<=value<=4095 for value in
            (a.center_x_raw,a.center_y_raw)):p.error('Choose valid ESP32 centre readings and tolerance')
    auto=a.task=='joystick' and not a.manual
    automatic_ready=auto or (a.task=='ball' and not a.manual)
    conditions=['baseline','eit']+(['kinematic'] if a.include_true_position else [])
    if a.axis is not None:configure_joystick_mechanism(a.mechanism,a.inward_arc_m,a.axis)
    elif a.mechanism is not None:configure_joystick_mechanism(a.mechanism,a.inward_arc_m)
    checked=check_controller(a.task,a.block,a.config_dir)
    if a.axis is not None and checked.get('joystick_axis')!=a.axis:
        raise RuntimeError('Controller axis does not match --axis; no motion started')
    if a.task=='joystick':validate_inward_configuration(checked,calibration)
    script=Path(__file__).with_name('experiment.py')
    created=datetime.now(timezone.utc)
    stamp=created.astimezone().strftime('%Y-%m-%d_%H-%M-%S_%f_%z')
    folder=a.output/a.task/(a.block+'_'+stamp)
    folder.mkdir(parents=True,exist_ok=False)
    if calibration is not None:
        shutil.copyfile(calibration_path,folder/'joystick_adc_calibration.json')
    (folder/'block.json').write_text(json.dumps(vars(a)|{'conditions':conditions,'output':str(a.output),'config_dir':str(config_directory(a.config_dir)),'created_utc':created.isoformat(),'automatic_joystick':auto,'automatic_ready':automatic_ready,'checked_controller_parameters':checked,
        'joystick_recorder_csv':str(folder/'joystick_esp32.csv') if a.task=='joystick' else None,
        'joystick_calibrated_csv':str(folder/'joystick_calibrated.csv') if a.task=='joystick' else None,
        'reproducibility_manifest':'reproducibility/manifest.json',
        'note':'Operator must launch the matching offset/config. Evaluate actual object motion, not controller done. Inward_arc uses the same calibrated ESP32 stream for feedback and recording.'},indent=2,default=str)+'\n')
    print('This runs the physical robot. Use the matching launch/config; record independent object motion.')
    print('Each repeat tests:',', '.join(conditions),'. Order is counterbalanced across repeats. Logs:',folder)
    if a.task=='joystick':print('Joystick Base tilt axis:',checked.get('joystick_axis','x'),flush=True)
    if a.task=='ball':
        axis=checked.get('ball_axis','x')
        initial=checked.get('initial_ball_'+axis,.45 if axis=='x' else -.10)
        distance=checked.get('roll_distance',.020)
        ratio=checked.get('ball_baseline_motion_ratio',1.)
        print(f'Ball Base {axis.upper()} goal: {initial:+.3f} -> {initial+distance:+.3f} m; '
              f'nominal palm stroke {distance/ratio:+.3f} m with '
              f'baseline ball/pad travel ratio {ratio:.2f}.',flush=True)
        print(f'Each condition retracts after sustained estimated-goal attainment or '
              f'{checked.get("ball_duration",8.):.1f} s, whichever comes first. '
              'Camera travel is recorded separately.',flush=True)
    if auto:print('Automatic joystick sequence: ready, approach, manipulate and retract for each condition; no keyboard prompts.',flush=True)
    elif automatic_ready:print('Ball ready motion starts automatically for each condition; the run prompt remains after ready.',flush=True)
    recorder=None
    recorder_log=None
    feedback=None
    result={'status':'running','stage':'recorders','repeat':None,'condition':None}
    try:
        result['stage']='recording_conditions'
        initialize_recording(folder,config_directory(a.config_dir),[sys.executable,*sys.argv],a.conditions_file)
        result['stage']='recorders'
        if a.task=='joystick':
            print('Starting ESP32 recorder before any robot motion...',flush=True)
            recorder,recorder_csv,recorder_log=start_joystick_recorder(folder,a.joystick_port)
            print('ESP32 samples:',recorder_csv,flush=True)
            feedback,feedback_log=start_shaft_feedback(folder,recorder_csv,
                checked.get('joystick_feedback_topic','/joystick/shaft'),a.center_x_raw,a.center_y_raw)
            if checked.get('joystick_mechanism')=='inward_arc' or a.include_true_position:
                wait_for_shaft_feedback(feedback,feedback_log)
                print('Live ESP32 shaft feedback ready for alignment/kinematic position.',flush=True)
        for repeat in range(1,a.repeats+1):
            order=condition_order(repeat,conditions)
            for condition in order:
                result.update(repeat=repeat,condition=condition,stage='ready')
                print(f'\n{a.task} / {a.block} / repeat {repeat}/{a.repeats} / {condition}')
                destination=folder/f'repeat_{repeat:02d}'
                if recorder:check_joystick_recorder(recorder,recorder_log)
                if feedback and feedback.poll() is not None:raise RuntimeError(f'Shaft feedback stopped; see {feedback_log}')
                if auto:
                    wait_for_joystick_center(recorder_csv,recorder,recorder_log,
                                             a.center_x_raw,a.center_y_raw,a.center_tolerance_raw,
                                             allow_tilt=checked.get('joystick_mechanism')=='inward_arc')
                elif not automatic_ready and require_command('Clear the local ready-motion route; type ready or r and press Enter (q stops): ',('ready','r')) is None:return
                command=[sys.executable,str(script),'--task',a.task,'--action','ready',
                         '--output',str(destination),'--record-root',str(folder)]
                record_command(folder,'ready',command,repeat=repeat,condition=condition)
                subprocess.run(command,check=True)
                if recorder:check_joystick_recorder(recorder,recorder_log)
                if feedback and feedback.poll() is not None:raise RuntimeError(f'Shaft feedback stopped; see {feedback_log}')
                if auto:
                    wait_for_joystick_center(recorder_csv,recorder,recorder_log,
                                             a.center_x_raw,a.center_y_raw,a.center_tolerance_raw,
                                             allow_tilt=checked.get('joystick_mechanism')=='inward_arc')
                elif a.task=='joystick':
                    print('Reset joystick upright; leave pad unloaded. ESP32 recording:',recorder_csv)
                else:
                    ball_axis=checked.get('ball_axis','x')
                    ball_start=checked.get('initial_ball_'+ball_axis,.45 if ball_axis=='x' else -.10)
                    print('Remove any ball stuck to the EIT pad. Reset the ball centre to its '
                          f'marked Base {ball_axis.upper()}={ball_start:+.3f} m start. '
                          'Leave the pad unloaded; start independent camera recording. '
                          'Pressing run/r will recalibrate the unloaded pad before approach.')
                if a.task=='ball' and a.block=='push':
                    print('During rolling apply ONE push using the marked displacement/timing jig; controller gets no push flag.')
                else:print('Do not apply an external push during this trial.')
                if not auto and require_command('Type run or r and press Enter to approach and manipulate (q stops): ',('run','r')) is None:return
                if recorder:check_joystick_recorder(recorder,recorder_log)
                if feedback and feedback.poll() is not None:raise RuntimeError(f'Shaft feedback stopped; see {feedback_log}')
                released=False
                slipped=False
                trial_error=None
                result['stage']='trial'
                try:
                    command=[sys.executable,str(script),'--task',a.task,'--action','trial',
                             '--condition',condition,'--output',str(destination),'--record-root',str(folder)]
                    record_command(folder,'trial',command,repeat=repeat,condition=condition)
                    subprocess.run(command,check=True)
                except subprocess.CalledProcessError as exc:
                    if a.task=='ball':
                        trial_error=exc
                        print('Ball controller stopped; record any camera-measured travel before ending this block.',flush=True)
                    elif exc.returncode==3:
                        released=True
                        print('Contact-release trial retracted and recorded as unsuccessful; proceeding to the next condition.',flush=True)
                    elif a.task=='joystick' and exc.returncode==4:
                        released=True;slipped=True
                        print('Joystick slip trial retracted and recorded as failed; proceeding to the next condition.',flush=True)
                    else:raise
                finally:
                    if recorder and not auto:plot_joystick_block(folder)
                if recorder:check_joystick_recorder(recorder,recorder_log)
                if feedback and feedback.poll() is not None:raise RuntimeError(f'Shaft feedback stopped; see {feedback_log}')
                if a.task=='joystick':
                    outcome={'success':False,'reason':('joystick_slip' if slipped else 'contact_release_snapback')} if released else None
                    status=('failed_joystick_slip' if slipped else 'unsuccessful_contact_release') if released else 'pending_esp32_review'
                    if not auto:print('ESP32 result saved for later review; continue with the next ready prompt.')
                else:
                    axis=checked.get('ball_axis','x')
                    initial=float(checked.get('initial_ball_'+axis,.45 if axis=='x' else -.10))
                    goal=initial+float(checked.get('roll_distance',.02))
                    controller_result=ball_trial_summary(destination,condition,axis,initial)
                    status,actual_displacement=camera_ball_displacement(axis)
                    if status=='quit':return
                    outcome=input('Camera notes (contact, slip, video file; optional): ').strip()
                    camera_result=dict(camera_ball_start_axis_m=initial,
                        camera_ball_displacement_m=actual_displacement,
                        camera_ball_final_axis_m=initial+actual_displacement if actual_displacement is not None else None,
                        camera_ball_relative_goal_axis_m=goal,
                        camera_ball_goal_error_m=initial+actual_displacement-goal if actual_displacement is not None else None,
                        camera_ball_absolute_goal_error_m=initial+actual_displacement-goal if actual_displacement is not None else None,
                        camera_ball_measurement_source='operator_from_camera' if actual_displacement is not None else None)
                with (folder/'outcomes.jsonl').open('a') as stream:
                    stream.write(json.dumps({'repeat':repeat,'condition':condition,'observation':outcome,
                        'observation_status':status,
                        **(controller_result|camera_result if a.task=='ball' else {}),
                        'trial_contact_released':released,
                        'joystick_slipped':slipped if a.task=='joystick' else None,
                        'joystick_recorder_csv':str(recorder_csv) if recorder else None,
                        'completed_utc':datetime.now(timezone.utc).isoformat()})+'\n')
                if trial_error is not None:raise trial_error
        print('Block finished. CSV files record controller execution; evaluate object success independently.')
        result['status']='completed'
    except KeyboardInterrupt:
        result.update(status='interrupted',error='KeyboardInterrupt')
        print('\nBlock interrupted. Inspect the controller/robot state before continuing.')
        raise SystemExit(130)
    except BaseException as exc:
        result.update(status='error',error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if result['status']=='running':result['status']='stopped_by_operator'
        result['finished_utc']=datetime.now(timezone.utc).isoformat()
        try:(folder/'block_result.json').write_text(json.dumps(result,indent=2)+'\n')
        except OSError as exc:print('Could not save block result: '+str(exc),file=sys.stderr,flush=True)
        cleanups=[]
        if auto and recorder:cleanups.append(lambda:stop_controller(script,a.task))
        if recorder:cleanups.append(lambda:stop_joystick_recorder(recorder))
        if feedback:cleanups.append(lambda:stop_joystick_recorder(feedback))
        for cleanup in cleanups:
            try:cleanup()
            except Exception as exc:print('Cleanup error: '+str(exc),file=sys.stderr,flush=True)
        if recorder:
            print('ESP32 recording saved:',recorder_csv,flush=True)
        if a.task=='joystick':
            plot_joystick_block(folder)
            render_joystick_block(folder)
            summary=folder/'run_summary.png'
            if a.show_plots and summary.is_file():
                try:
                    subprocess.Popen(['xdg-open',str(summary)],stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL,start_new_session=True)
                except OSError as exc:
                    print('Could not open saved summary: '+str(exc),file=sys.stderr,flush=True)
        elif a.task=='ball':
            try:write_ball_comparison(folder)
            except Exception as exc:print('Could not save ball comparison: '+str(exc),file=sys.stderr,flush=True)
            render_ball_block(folder)


if __name__=='__main__':
    try:main()
    except RuntimeError as exc:
        print('Block stopped: '+str(exc),file=sys.stderr)
        raise SystemExit(1)
