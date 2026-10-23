#!/usr/bin/env python3
"""Capture q, including q7, from the existing controller's ROS JointState topic."""
import argparse,time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--topic',default='/joint_states');p.add_argument('--prefix',default='fr3_');p.add_argument('--output',required=True,type=Path)
    a,ros=p.parse_known_args();rclpy.init(args=ros);node=Node('eit_capture_posture');result=[]
    names=[a.prefix+'joint'+str(i) for i in range(1,8)]
    def callback(message):
        mapping=dict(zip(message.name,message.position))
        if all(n in mapping for n in names):result[:]=[float(mapping[n]) for n in names]
    sub=node.create_subscription(JointState,a.topic,callback,10)
    try:
        deadline=time.monotonic()+10
        while rclpy.ok() and not result and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if len(result)!=7:raise RuntimeError('No complete seven-joint posture; check topic and prefix')
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'posture_reference_rad':result,'use_saved_posture':True}}},sort_keys=False))
        print('Saved posture, q7 =',result[-1],'rad:',a.output)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
