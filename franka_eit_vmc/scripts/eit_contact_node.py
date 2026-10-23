#!/usr/bin/env python3
"""Publish physical-pad contact state and position from Amodo ARIA phase frames."""
import argparse
import json
import threading
import time
from collections import deque
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from franka_eit_vmc.msg import EitContact
from franka_eit_tools.eit_reader import EITReader,parse_frame
from franka_eit_tools.inference import Inference

class SensorNode(Node):
    def __init__(self,args):
        super().__init__('eit_contact')
        self.engine=Inference(args.model)
        cfg=json.loads(Path(args.config).read_text())
        trained=self.engine.model.metadata.get('eit_config',{})
        keys=['eit_electrode_pins','eit_frequency_khz','eit_periods_per_measurement']
        for key in keys:
            if key not in trained:raise ValueError('Model is missing physical EIT configuration: '+key)
            if cfg.get(key) is not None and cfg[key]!=trained[key]:
                raise ValueError('Sensor configuration differs from training: '+key)
            cfg[key]=trained[key]
        self.reader=EITReader(cfg);self.frame_id=args.frame;self.stop=threading.Event()
        self.baseline_requested=threading.Event();self.frames=deque(maxlen=20)
        self.sequence=0;self.failed=None;self.output=deque(maxlen=100)
        self.publisher=self.create_publisher(EitContact,'/eit/contact',10)
        self.service=self.create_service(Trigger,'~/calibrate_baseline',self.calibrate)
        self.timer=self.create_timer(.01,self.publish)
        self.thread=threading.Thread(target=self.receive,daemon=True);self.thread.start()
        self.get_logger().info('Keep the pad unloaded, then call /eit_contact/calibrate_baseline. No contact is published until calibration completes.')
        if not self.engine.physical:self.get_logger().warning('DEMO model: robot experiment controllers reject these estimates.')

    def calibrate(self,request,response):
        del request
        if self.failed:
            response.success=False;response.message=str(self.failed)
        elif self.engine.contact:
            response.success=False;response.message='Release contact before recalibrating the baseline'
        else:
            self.baseline_requested.set();response.success=True;response.message='Queued: leave pad unloaded for 20 fresh electrical frames'
        return response

    def receive(self):
        try:
            self.reader.open()
            collecting=False;baseline=[]
            while not self.stop.is_set():
                if self.reader.ser.in_waiting>10000 or self.reader._rx_buffer.count(b'\n')>3:
                    raise RuntimeError('EIT serial backlog is too large to timestamp frames as fresh')
                line=self.reader._line(time.monotonic()+2.)
                received_ns=time.time_ns()
                if line.startswith('ERROR:'):raise RuntimeError(line)
                try:frame=parse_frame(line)
                except ValueError:
                    self.sequence+=1
                    self.output.append((received_ns,self.sequence,{'contact':False,'measurement_valid':False,'position_valid':False,'x_m':0.,'y_m':0.,'signal_strength':0.,'numbering':self.engine.model.metadata['numbering'],'model_is_physical':False,'model_sha256':self.engine.sha256}))
                    continue
                if self.baseline_requested.is_set():
                    self.baseline_requested.clear();collecting=True;baseline=[];self.engine.baseline=None
                if collecting:
                    baseline.append(frame)
                    if len(baseline)>=20:
                        self.engine.set_baseline({'frames':baseline});collecting=False
                self.sequence+=1
                self.output.append((received_ns,self.sequence,self.engine.step(frame)))
        except Exception as exc:
            self.failed=exc
        finally:
            self.reader.close()

    def publish(self):
        if self.failed:
            self.get_logger().error('EIT stream stopped: '+str(self.failed));self.timer.cancel();return
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
    parser.add_argument('--model',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
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
