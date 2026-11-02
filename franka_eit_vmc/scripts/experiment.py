#!/usr/bin/env python3
"""Operator heartbeat, ready/start services, and timestamped per-attempt CSV recording."""
import argparse,csv,math,time,sys
from datetime import datetime,timezone
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger,SetBool
from franka_eit_vmc.msg import ExperimentState,EitContact
from franka_eit_tools.recording import initialize_recording,snapshot,record_parameter_event
from rcl_interfaces.msg import ParameterEvent
from rclpy.qos import qos_profile_parameter_events

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=['joystick','ball'],required=True)
    p.add_argument('--action',choices=['ready','trial','stop'],required=True)
    p.add_argument('--condition',choices=['eit','baseline','kinematic'],default='eit')
    p.add_argument('--output',type=Path,default=Path('trial_logs'))
    p.add_argument('--record-root',type=Path,help='Block folder for shared reproducibility artifacts; defaults to --output')
    p.add_argument('--conditions-file',type=Path,help='Optional physical setup / object / camera notes to archive')
    a,ros=p.parse_known_args();name='/eit_joystick' if a.task=='joystick' else '/eit_ball_roll'
    if a.task!='joystick' and a.condition=='kinematic':p.error('Kinematic tip position applies only to joystick trials')
    rclpy.init(args=ros);node=Node('eit_experiment_operator')
    heartbeat=node.create_publisher(Bool,name+'/heartbeat',10)
    timer=node.create_timer(.05,lambda:heartbeat.publish(Bool(data=True)))
    heartbeat.publish(Bool(data=True))
    latest=[None];rows=[]
    def callback(message):
        latest[0]=message;rows.append(message)
    sub=node.create_subscription(ExperimentState,name+'/status',callback,100)
    sensor=[None]
    sensor_sub=node.create_subscription(EitContact,'/eit/contact',lambda message:sensor.__setitem__(0,message),10)
    def call(kind,suffix,request,timeout=5.):
        service=suffix if suffix.startswith('/') else name+'/'+suffix
        client=node.create_client(kind,service)
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Service unavailable: '+suffix)
        future=client.call_async(request);deadline=time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        if not future.done() or future.result() is None:raise RuntimeError('Service timed out: '+suffix)
        result=future.result()
        if not result.success:raise RuntimeError(result.message)
        return result
    mechanism_fields=['ball_normal_position_m','ball_normal_velocity_m_s','ball_normal_origin_m',
        'ball_normal_force_n','ball_estimated_velocity_m_s','ball_drive_reference_m',
        'ball_normal_brake','ball_normal_at_stop',
        'ball_recover_contact','ball_contact_ready','ball_contact_paused','ball_goal_reached','ball_contact_loss_duration_s',
        'ball_loaded_contact_duration_s','approach_distance_limit_m','contact_source','ball_axis','ball_goal_y_m','ball_goal_coordinate_m',
        'trial_contact_released','joystick_slipped','joystick_mechanism','joystick_axis','joystick_inward_arc_m',
        'joystick_shaft_angle_rad','joystick_shaft_rate_rad_s',
        'joystick_shaft_angle_y_rad','joystick_shaft_rate_y_rad_s','joystick_shaft_age_s',
        'joystick_shaft_fresh','joystick_shaft_clipped','joystick_force_scale',
        'joystick_task_spring_scale','joystick_alignment_min_spring_scale','joystick_slider_at_stop']
    log=None;completed=False;record_tag=None
    record_root=a.record_root or a.output
    def capture_record(stage,require_baseline=False):
        return snapshot(node,record_root,record_tag+'.'+stage,a.task,
            dict(task=a.task,action=a.action,condition=a.condition if a.action=='trial' else None,
                 stage=stage,command=[sys.executable,*sys.argv],
                 trace=str(log.name) if log else None,controller_status=latest[0],eit_status=sensor[0]),require_baseline)
    def calibrate_unloaded():
        calibration=call(Trigger,'/eit_contact/calibrate_baseline',Trigger.Request(),timeout=25.)
        if latest[0].fault or latest[0].mode_name not in ('hold','done'):
            raise RuntimeError('Controller left its stationary pose during baseline calibration')
        calibrated_at_ns=time.time_ns()
        sensor[0]=None
        sensor_deadline=time.monotonic()+5.
        while rclpy.ok() and time.monotonic()<sensor_deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            reading=sensor[0]
            if (reading and reading.model_is_physical and reading.measurement_valid
                    and not reading.contact and
                    reading.header.stamp.sec*1_000_000_000+reading.header.stamp.nanosec>=calibrated_at_ns):
                print(calibration.message,flush=True)
                return
        raise RuntimeError('No fresh EIT non-contact reading after baseline calibration')
    try:
        deadline=time.monotonic()+3.
        while rclpy.ok() and latest[0] is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if latest[0] is None:raise RuntimeError('No controller status received')
        if a.action=='stop':
            call(Trigger,'stop',Trigger.Request());completed=True;return
        if a.action=='trial':
            if a.condition=='kinematic':call(Trigger,'use_kinematic_position',Trigger.Request())
            else:call(SetBool,'use_eit',SetBool.Request(data=a.condition=='eit'))
            deadline=time.monotonic()+3.
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
                if latest[0].contact_source==a.condition and latest[0].mode_name in ('hold','done'):break
            else:raise RuntimeError('Controller did not acknowledge contact source: '+a.condition)
        if a.action in ('ready','trial'):
            a.output.mkdir(parents=True,exist_ok=True)
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            label=a.condition if a.action=='trial' else 'ready'
            log=(a.output/f'{a.task}_{label}_{stamp}.csv').open('x',newline='')
            writer=csv.writer(log)
            writer.writerow(['host_time_ns','mode','fault','use_eit','retract_incomplete','trial_time_s','progress_m','progress_velocity_m_s','ball_goal_x_m','theta_d_rad','theta_d_y_rad','tracking_error_m',
                             'sensor_sequence','sensor_contact','sensor_contact_candidate','sensor_fresh','sensor_measurement_valid','sensor_position_valid','sensor_physical_model','sensor_age_s','sensor_signal_strength','sensor_contact_threshold','sensor_pattern_distance','sensor_pattern_support_limit','sensor_predicted_radius_mm']+
                            [f'{prefix}_{axis}' for prefix in ['sensor_contact_pad_m','sensor_contact_base_m','pad_m','pad_normal_base','desired_pad_normal_base','point_m','reference_m','attachment_m','coupling_force_N','requested_force_N','requested_moment_Nm','external_force_N','external_moment_Nm'] for axis in ['x','y','z']]+[f'q{i}_rad' for i in range(1,8)]+[f'tau{i}_Nm' for i in range(1,8)]+mechanism_fields+['joystick_slider_u_m','joystick_slider_v_m']+[f'{prefix}_{axis}' for prefix in ['alignment_force_N','alignment_centre_m','alignment_target_m','tool_weight_compensation_force_N','joystick_tip_position_m'] for axis in ['x','y','z']])
            print('Controller trace:',log.name)
            record_tag=Path(log.name).stem
            initialize_recording(record_root,command=[sys.executable,*sys.argv],conditions_file=a.conditions_file)
            def parameter_event(event):
                if event.node in (name,'/eit_contact','/eit_joystick_shaft_feedback'):
                    record_parameter_event(record_root,event)
            event_sub=node.create_subscription(ParameterEvent,'/parameter_events',parameter_event,
                                               qos_profile_parameter_events)
            if a.task=='ball' and a.action=='trial':
                print('Calibrating unloaded EIT baseline after run command, before approach...',flush=True)
                calibrate_unloaded()
            capture_record('before_motion',require_baseline=a.action=='trial')
        rows.clear()
        start_service='move_ready' if a.action=='ready' else 'start_trial'
        heartbeat_deadline=time.monotonic()+3.
        while True:
            try:
                call(Trigger,start_service,Trigger.Request())
                break
            except RuntimeError as exc:
                if str(exc)!='Operator heartbeat not received' or time.monotonic()>=heartbeat_deadline:
                    raise
                rclpy.spin_once(node,timeout_sec=.05)
        print('Command queued. Ctrl+C requests a bounded stop. CSV completion is controller execution, not externally measured task success.')
        deadline=time.monotonic()+(300. if a.action=='ready' else 45.)
        start_deadline=time.monotonic()+3.
        active_seen=False
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            while rows:
                message=rows.pop(0)
                if log:
                    tracking_error=math.dist(message.pad_position_m,message.reference_m)
                    writer.writerow([time.time_ns(),message.mode_name,message.fault_name,message.use_eit,message.retract_incomplete,message.trial_time_s,message.progress_m,message.progress_velocity_m_s,message.ball_goal_x_m,message.joystick_angle_command_rad,message.joystick_angle_command_y_rad,tracking_error,
                                     message.sensor_sequence,message.sensor_contact,message.sensor_contact_candidate,message.sensor_fresh,message.sensor_measurement_valid,message.sensor_position_valid,message.sensor_physical_model,message.sensor_age_s,message.sensor_signal_strength,message.sensor_contact_threshold,message.sensor_pattern_distance,message.sensor_pattern_support_limit,message.sensor_predicted_radius_mm]+
                                    list(message.sensor_contact_pad_m)+list(message.sensor_contact_base_m)+list(message.pad_position_m)+list(message.pad_normal_base)+list(message.desired_pad_normal_base)+list(message.controlled_point_m)+list(message.reference_m)+list(message.attachment_in_pad_m)+list(message.coupling_force_n)+list(message.requested_cartesian_force_n)+list(message.requested_cartesian_moment_nm)+list(message.external_force_n)+list(message.external_moment_nm)+list(message.joint_position_rad)+list(message.commanded_torque_nm)+[getattr(message,name) for name in mechanism_fields]+list(message.joystick_slider_m)+list(message.joystick_alignment_force_n)+list(message.joystick_alignment_centre_m)+list(message.joystick_alignment_target_m)+list(message.tool_weight_compensation_force_n)+list(message.joystick_tip_position_m))
                if message.fault:
                    detail=(f' (pad Z={message.pad_position_m[2]:.3f} m, '
                            f'reference Z={message.reference_m[2]:.3f} m, '
                            f'command Z={message.requested_cartesian_force_n[2]:.1f} N; '
                            f'EIT contact={message.sensor_contact}, fresh={message.sensor_fresh}, '
                            f'measurement_valid={message.sensor_measurement_valid}, '
                            f'candidate={message.sensor_contact_candidate}, '
                            f'signal={message.sensor_signal_strength:.6f}, '
                            f'threshold={message.sensor_contact_threshold:.6f}, '
                            f'pattern={message.sensor_pattern_distance:.3f}/{message.sensor_pattern_support_limit:.3f}, '
                            f'radius={message.sensor_predicted_radius_mm:.1f} mm, '
                            f'age={message.sensor_age_s:.3f} s)')
                    raise RuntimeError('Controller stopped: '+message.fault_name+detail)
                if message.mode_name not in ['hold','done']:active_seen=True
                if active_seen and ((a.action=='ready' and message.mode_name=='hold') or (a.action=='trial' and message.mode_name=='done')):
                    if a.action=='ready':
                        if a.task=='joystick':
                            print('Ready pose reached; calibrating unloaded EIT baseline...',flush=True)
                            calibrate_unloaded()
                            capture_record('after_ready_calibration',require_baseline=True)
                    if a.action=='trial' and message.trial_contact_released and message.retract_incomplete:
                        raise RuntimeError('Contact-release trial could not finish retraction; inspect clearance before continuing')
                    completed=True
                    if a.action=='trial' and getattr(message,'joystick_slipped',False):
                        print('Trial failed: joystick slip detected. Retraction completed; continuing to the next condition.',flush=True)
                        # Reserved result consumed by run_block.py, distinct from a controller fault.
                        raise SystemExit(4)
                    if a.action=='trial' and message.trial_contact_released:
                        print('Trial unsuccessful: contact release with shaft snapback. Retraction completed; continuing to the next condition.',flush=True)
                        # Reserved result consumed by run_block.py, distinct from a controller fault.
                        raise SystemExit(3)
                    if a.action=='trial' and message.retract_incomplete:
                        print('Manipulation finished; retract incomplete while EIT reported no contact. Confirm the next ready route before moving.')
                    else:
                        if a.task=='ball' and a.action=='trial':
                            reason='estimated goal reached' if message.ball_goal_reached else 'time limit reached'
                            print(f'Finished: trial ({reason}; camera travel is recorded separately)')
                        else:print('Finished:',a.action)
                    return
            if not active_seen and time.monotonic()>start_deadline:
                raise RuntimeError('Controller did not start the queued command within 3 s')
        raise RuntimeError('Operation timed out; requesting stop')
    except KeyboardInterrupt:
        print('Interrupted; requesting stop')
        raise SystemExit(130)
    finally:
        # A reported controller fault already stops its FCI loop. Its service
        # normally exits before cleanup can call stop or take another snapshot.
        controller_faulted=bool(latest[0] and latest[0].fault)
        try:
            if rclpy.ok() and not completed and not controller_faulted:
                call(Trigger,'stop',Trigger.Request())
        except Exception as exc:print('Stop request:',exc)
        if log:log.flush();log.close()
        if record_tag and controller_faulted:
            print('Controller exited after its fault; pre-motion conditions and trace are saved. '
                  'A final controller snapshot is unavailable.',file=sys.stderr)
        elif record_tag:
            try:capture_record('after_operation')
            except Exception as exc:print('Final conditions snapshot incomplete:',exc,file=sys.stderr)
        timer.cancel();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
