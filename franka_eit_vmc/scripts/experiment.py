#!/usr/bin/env python3
"""Operator heartbeat, ready/start services, and timestamped per-attempt CSV recording."""
import argparse,csv,time
from datetime import datetime,timezone
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger,SetBool
from franka_eit_vmc.msg import ExperimentState

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=['joystick','ball'],required=True)
    p.add_argument('--action',choices=['ready','trial','stop'],required=True)
    p.add_argument('--condition',choices=['eit','baseline'],default='eit')
    p.add_argument('--output',type=Path,default=Path('trial_logs'))
    a,ros=p.parse_known_args();name='/eit_joystick' if a.task=='joystick' else '/eit_ball_roll'
    rclpy.init(args=ros);node=Node('eit_experiment_operator')
    heartbeat=node.create_publisher(Bool,name+'/heartbeat',10)
    timer=node.create_timer(.05,lambda:heartbeat.publish(Bool(data=True)))
    latest=[None];rows=[]
    def callback(message):
        latest[0]=message;rows.append(message)
    sub=node.create_subscription(ExperimentState,name+'/status',callback,100)
    def call(kind,suffix,request):
        client=node.create_client(kind,name+'/'+suffix)
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Service unavailable: '+suffix)
        future=client.call_async(request);deadline=time.monotonic()+5.
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        if not future.done() or future.result() is None:raise RuntimeError('Service timed out: '+suffix)
        result=future.result()
        if not result.success:raise RuntimeError(result.message)
        return result
    log=None;completed=False
    try:
        deadline=time.monotonic()+3.
        while rclpy.ok() and latest[0] is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if latest[0] is None:raise RuntimeError('No controller status received')
        if a.action=='stop':
            call(Trigger,'stop',Trigger.Request());completed=True;return
        if a.action=='trial':
            call(SetBool,'use_eit',SetBool.Request(data=a.condition=='eit'))
            a.output.mkdir(parents=True,exist_ok=True)
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            log=(a.output/f'{a.task}_{a.condition}_{stamp}.csv').open('x',newline='')
            writer=csv.writer(log)
            writer.writerow(['host_time_ns','mode','fault','use_eit','trial_time_s','progress_m','progress_velocity_m_s','ball_goal_x_m','theta_d_rad']+[f'{prefix}_{axis}' for prefix in ['pad_m','point_m','reference_m','attachment_m','coupling_force_N','external_force_N'] for axis in ['x','y','z']]+[f'q{i}_rad' for i in range(1,8)]+[f'tau{i}_Nm' for i in range(1,8)])
        rows.clear();call(Trigger,'move_ready' if a.action=='ready' else 'start_trial',Trigger.Request())
        print('Command queued. Ctrl+C requests a bounded stop. CSV completion is controller execution, not externally measured task success.')
        deadline=time.monotonic()+(300. if a.action=='ready' else 45.)
        active_seen=False
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            while rows:
                message=rows.pop(0)
                if log:
                    writer.writerow([time.time_ns(),message.mode_name,message.fault_name,message.use_eit,message.trial_time_s,message.progress_m,message.progress_velocity_m_s,message.ball_goal_x_m,message.joystick_angle_command_rad]+list(message.pad_position_m)+list(message.controlled_point_m)+list(message.reference_m)+list(message.attachment_in_pad_m)+list(message.coupling_force_n)+list(message.external_force_n)+list(message.joint_position_rad)+list(message.commanded_torque_nm))
                if message.fault:
                    raise RuntimeError('Controller stopped: '+message.fault_name)
                if message.mode_name not in ['hold','done']:active_seen=True
                if active_seen and ((a.action=='ready' and message.mode_name=='hold') or (a.action=='trial' and message.mode_name=='done')):
                    completed=True;print('Finished:',a.action);return
        raise RuntimeError('Operation timed out; requesting stop')
    except KeyboardInterrupt:print('Interrupted; requesting stop')
    finally:
        try:
            if rclpy.ok() and not completed:call(Trigger,'stop',Trigger.Request())
        except Exception as exc:print('Stop request:',exc)
        if log:log.flush();log.close()
        timer.cancel();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
