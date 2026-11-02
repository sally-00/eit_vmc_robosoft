#!/usr/bin/env python3
"""Publish physical-pad contact state and position from Amodo ARIA phase frames."""
import argparse
import json
import copy
import os
import signal
import stat
import threading
import time
from collections import deque
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from franka_eit_vmc.msg import EitContact
from franka_eit_tools.eit_reader import EITReader,parse_frame
from franka_eit_tools.inference import Inference
from franka_eit_tools.model_selection import select_model,eit_config_path
from franka_eit_tools.recording import file_packet,python_packets,json_default,process_identity

MODEL_SCAN_KEYS=('eit_electrode_pins','eit_drive_pattern','eit_frequency_khz',
                 'eit_periods_per_measurement','eit_rx_gain_margin_steps')
ROBOT_EXECUTABLES={'ball_roll_controller','joystick_controller','move_ready'}

def process_arguments(pid):
    try:
        return [os.fsdecode(arg) for arg in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if arg]
    except OSError:
        return []

def process_ids():
    return [int(path.name) for path in Path('/proc').iterdir() if path.name.isdigit()]

def serial_port_owners(port):
    """Find processes with the same character device open, including symlinked paths."""
    try:device=os.stat(port)
    except OSError:return []
    if not stat.S_ISCHR(device.st_mode):return []
    owners=[]
    for pid in process_ids():
        if pid==os.getpid():continue
        try:
            descriptors=list(Path(f'/proc/{pid}/fd').iterdir())
        except OSError:
            continue  # The process exited or belongs to another user.
        for fd in descriptors:
            try:info=fd.stat()
            except OSError:continue
            if stat.S_ISCHR(info.st_mode) and info.st_rdev==device.st_rdev:
                owners.append(pid)
                break
    return owners

def our_eit_node(arguments):
    return any(Path(arg).name=='eit_contact_node.py' and
               'franka_eit_vmc' in Path(arg).parts for arg in arguments)

def robot_controller_running():
    return any(any(Path(arg).name in ROBOT_EXECUTABLES and
                   'franka_eit_vmc' in Path(arg).parts for arg in process_arguments(pid))
               for pid in process_ids() if pid!=os.getpid())

def release_previous_eit_node(port,logger):
    """Gracefully replace only a prior copy of this sensor node on this port."""
    if not port:return
    owners=serial_port_owners(port)
    if not owners:return
    if robot_controller_running():
        raise RuntimeError(f'{port} is busy and a Franka controller is running; stop the controller before replacing its EIT sensor')
    previous=[]
    for pid in owners:
        try:same_user=Path(f'/proc/{pid}').stat().st_uid==os.getuid()
        except OSError:continue
        arguments=process_arguments(pid)
        if not our_eit_node(arguments) or not same_user:
            raise RuntimeError(f'{port} is held by PID {pid} ({" ".join(arguments) or "unknown process"}); close that program before starting the EIT node')
        previous.append(pid)
    for pid in previous:
        # Recheck the port and command immediately before signalling a PID.
        if pid not in serial_port_owners(port) or not our_eit_node(process_arguments(pid)):
            continue
        logger.info(f'Replacing previous EIT node PID {pid} on {port}')
        try:os.kill(pid,signal.SIGINT)
        except ProcessLookupError:continue
    deadline=time.monotonic()+5.
    while time.monotonic()<deadline:
        if not serial_port_owners(port):return
        time.sleep(.05)
    raise RuntimeError(f'{port} is still in use after requesting the previous EIT node to stop')

def acquisition_config_for_model(host_config,model):
    """Use training-time scan settings while retaining local port and detector settings."""
    config=copy.deepcopy(host_config)
    trained=model.metadata.get('eit_config',{})
    for key in MODEL_SCAN_KEYS:
        if key not in trained:
            raise ValueError('Model is missing physical EIT configuration: '+key)
        config[key]=copy.deepcopy(trained[key])
    reader=EITReader(config)
    if reader.protocol!=model.protocol or len(reader.protocol)!=model.channels:
        raise ValueError('Selected model and EIT reader have different measurement channels')
    return config,reader

class SensorNode(Node):
    def __init__(self,args):
        super().__init__('eit_contact')
        model_path=select_model(args.model)
        config_path=eit_config_path(args.config)
        host_cfg=json.loads(config_path.read_text())
        threshold=self.declare_parameter('contact_threshold',float(host_cfg.get('contact_threshold',0.0))).value
        self.engine=Inference(model_path,contact_threshold=threshold)
        self.state_lock=threading.RLock()
        self.recording_artifacts=[file_packet(model_path,'loaded_eit_model'),
                                  file_packet(config_path,'loaded_eit_config')]+python_packets(__file__)
        if self.recording_artifacts[0]['sha256']!=self.engine.sha256:
            raise RuntimeError('EIT model changed while loading')
        self.baseline_time_ns=None
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info(f'EIT model: {model_path} (SHA-256 {self.engine.sha256})')
        cfg,self.reader=acquisition_config_for_model(host_cfg,self.engine.model)
        changes=[f'{key}: {host_cfg.get(key)!r} -> {cfg[key]!r}' for key in MODEL_SCAN_KEYS
                 if host_cfg.get(key)!=cfg[key]]
        if changes:self.get_logger().info('Using selected model scan settings: '+', '.join(changes))
        self.get_logger().info(f'EIT scan: {cfg["eit_drive_pattern"]}, {len(self.reader.protocol)} measurements/frame')
        release_previous_eit_node(getattr(self.reader,'port',None),self.get_logger())
        self.frame_id=args.frame;self.stop=threading.Event()
        self.effective_config=copy.deepcopy(cfg)
        self.baseline_requested=threading.Event();self.baseline_complete=threading.Event();self.frames=deque(maxlen=20)
        self.startup_baseline_pending=True
        self.baseline_requested.set()  # One automatic unloaded capture after the first usable connection.
        self.sequence=0;self.failed=None;self.output=deque(maxlen=100)
        self.publisher=self.create_publisher(EitContact,'/eit/contact',10)
        self.service=self.create_service(Trigger,'~/calibrate_baseline',self.calibrate)
        self.recording_service=self.create_service(Trigger,'~/recording_state',self.recording_state)
        self.timer=self.create_timer(.01,self.publish)
        self.thread=threading.Thread(target=self.receive,daemon=True);self.thread.start()
        self.get_logger().info('Keep the pad unloaded: collecting 20 fresh electrical frames for automatic startup calibration. Joystick ready and ball trial start recalibrate again.')
        if not self.engine.physical:self.get_logger().warning('DEMO model: robot experiment controllers reject these estimates.')

    def on_parameters(self,parameters):
        for parameter in parameters:
            if parameter.name=='contact_threshold':
                if parameter.type_!=Parameter.Type.DOUBLE:
                    return SetParametersResult(successful=False,reason='contact_threshold must be a nonnegative finite number')
                try:
                    with self.state_lock:self.engine.set_contact_threshold(parameter.value)
                except ValueError as exc:return SetParametersResult(successful=False,reason=str(exc))
        return SetParametersResult(successful=True)

    def recording_state(self,request,response):
        del request
        with self.state_lock:
            state=dict(**process_identity(),frame_id=self.frame_id,
                effective_config=self.effective_config,acquisition=self.reader.metadata(),
                model_sha256=self.engine.sha256,model_metadata=self.engine.model.metadata,
                baseline=copy.deepcopy(self.engine.baseline),baseline_time_ns=self.baseline_time_ns,
                startup_baseline_pending=self.startup_baseline_pending,
                contact_threshold=self.engine.contact_threshold,
                hysteresis=dict(on_frames=self.engine.on_frames,off_frames=self.engine.off_frames,
                                on_count=self.engine.on,off_count=self.engine.off,contact=self.engine.contact),
                sequence=self.sequence,failed=str(self.failed) if self.failed else None,
                artifacts=self.recording_artifacts)
        response.success=True
        response.message=json.dumps(state,default=json_default,allow_nan=False)
        return response

    def calibrate(self,request,response):
        del request
        if self.failed:
            response.success=False;response.message=str(self.failed)
        else:
            self.baseline_complete.clear()
            self.baseline_requested.set()
            deadline=time.monotonic()+20.
            while not self.baseline_complete.wait(.05) and time.monotonic()<deadline:
                if self.failed:break
            response.success=self.baseline_complete.is_set()
            if response.success:
                response.message='Baseline calibrated from 20 fresh electrical frames; pad must be unloaded'
            elif self.failed:
                response.message='EIT baseline calibration failed: '+str(self.failed)
            else:
                response.message='EIT baseline calibration timed out after 20 s'
        return response

    def receive(self):
        while not self.stop.is_set():
            try:
                self.reader.open()
                self.failed=None
                self.get_logger().info('EIT stream connected. Collecting the unloaded startup baseline.'
                                       if self.startup_baseline_pending else
                                       'EIT stream reconnected. Calibrate the unloaded baseline before using contact estimates.')
                collecting=False;baseline=[]
                while not self.stop.is_set():
                    if self.reader.ser.in_waiting>10000 or self.reader._rx_buffer.count(b'\n')>3:
                        raise RuntimeError('EIT serial backlog is too large to timestamp frames as fresh')
                    line=self.reader._line(time.monotonic()+2.)
                    received_ns=time.time_ns()
                    if line.startswith('ERROR:'):raise RuntimeError(line)
                    try:frame=parse_frame(line,len(self.reader.protocol))
                    except ValueError:
                        self.sequence+=1
                        self.output.append((received_ns,self.sequence,{'contact':False,'contact_candidate':False,'measurement_valid':False,'position_valid':False,'x_m':0.,'y_m':0.,'signal_strength':0.,'contact_threshold':self.engine.contact_threshold,'pattern_distance':0.,'pattern_support_limit':self.engine.model.support_limit,'predicted_radius_mm':0.,'numbering':self.engine.model.metadata['numbering'],'model_is_physical':False,'model_sha256':self.engine.sha256}))
                        continue
                    if self.baseline_requested.is_set():
                        self.baseline_requested.clear();collecting=True;baseline=[]
                        self.output.clear()
                        with self.state_lock:
                            self.engine.baseline=None;self.baseline_time_ns=None
                            self.engine.on=0;self.engine.off=0;self.engine.contact=False
                    if collecting:
                        baseline.append(frame)
                        if len(baseline)>=20:
                            with self.state_lock:
                                self.engine.set_baseline({'frames':baseline})
                                self.baseline_time_ns=received_ns
                                startup=self.startup_baseline_pending
                                self.startup_baseline_pending=False
                            collecting=False
                            self.baseline_complete.set()
                            if startup:self.get_logger().info('Automatic startup baseline calibrated from 20 fresh frames. Recalibrate with the pad unloaded before the next trial.')
                    self.sequence+=1
                    with self.state_lock:prediction=self.engine.step(frame)
                    self.output.append((received_ns,self.sequence,prediction))
            except Exception as exc:
                if not self.stop.is_set():
                    if self.failed is None or str(self.failed)!=str(exc):
                        self.get_logger().error('EIT stream stopped: '+str(exc)+'; closing port and retrying in 2 s')
                    self.failed=exc
                    self.output.clear()
                    if self.startup_baseline_pending:self.baseline_requested.set()
                    else:self.baseline_requested.clear()
                    with self.state_lock:
                        self.engine.baseline=None;self.baseline_time_ns=None
                        self.engine.on=0;self.engine.off=0;self.engine.contact=False
            finally:
                try:self.reader.close()
                except Exception as exc:
                    if not self.stop.is_set():self.get_logger().warning('Could not cleanly close EIT port: '+str(exc))
            if self.stop.wait(2.):break

    def publish(self):
        if self.failed:return
        while self.output:
            stamp,sequence,pred=self.output.popleft();message=EitContact()
            message.header.stamp.sec=stamp//1000000000;message.header.stamp.nanosec=stamp%1000000000
            message.header.frame_id=self.frame_id;message.sequence=sequence
            for name,value in pred.items():setattr(message,name,value)
            # No physical-model acknowledgement until an unloaded baseline exists.
            message.model_is_physical=message.model_is_physical and self.engine.baseline is not None
            self.publisher.publish(message)

    def close(self):
        self.stop.set();self.thread.join(timeout=3.)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,help='Override the newest model in contact_position_runs')
    parser.add_argument('--config',type=Path,help='EIT settings JSON; defaults to EIT_CONFIG_DIR/eit.json or this checkout')
    parser.add_argument('--frame',default='eit_pad')
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args)
    node=None
    try:
        node=SensorNode(args);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
