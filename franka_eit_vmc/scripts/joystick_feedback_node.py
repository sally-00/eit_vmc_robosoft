#!/usr/bin/env python3
"""Publish live shaft feedback by tailing the block recorder; does not own USB."""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from franka_eit_vmc.msg import JoystickState
from franka_eit_tools.joystick import ShaftDecoder


class FeedbackNode(Node):
    def __init__(self, args):
        super().__init__('eit_joystick_shaft_feedback')
        calibration=json.loads(args.calibration.read_text())
        channel=calibration['mapping']['base_x_deg']['channel']
        center=args.center_x_raw if channel=='x_raw' else args.center_y_raw
        self.decoder = ShaftDecoder(calibration,observed_center=center)
        self.output=args.output.open('x',newline='') if args.output else None
        self.writer=csv.writer(self.output) if self.output else None
        if self.writer:
            self.writer.writerow(['host_unix_ns','host_monotonic_ns','esp_ms','x_raw','y_raw','pressed',
                'shaft_angle_rad','shaft_angle_deg','shaft_rate_rad_s','measurement_valid','clipped'])
        self.stream = args.csv.open()
        self.columns = None
        self.pending = ''
        self.sequence = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(JoystickState, args.topic, qos)
        self.timer = self.create_timer(.01, self.read_latest)

    def read_latest(self):
        # Bound each timer's work and publish the newest complete sample only.
        chunk=self.stream.read(262144)
        self.pending += chunk
        lines = self.pending.split('\n')
        self.pending = lines.pop()
        if len(self.pending) > 4096:
            raise RuntimeError('Malformed joystick recording: no bounded complete line')
        latest = None
        for line in lines:
            if not line.strip():
                continue
            fields = next(csv.reader(io.StringIO(line)))
            if self.columns is None:
                self.columns = fields
                if not {'host_unix_ns', 'host_monotonic_ns', self.decoder.channel} <= set(fields):
                    raise ValueError('Joystick recording lacks required feedback columns')
                continue
            if len(fields) != len(self.columns):
                continue
            try:
                row=dict(zip(self.columns,fields))
                latest = self.decoder.decode(row)
                if self.writer:
                    self.writer.writerow([row.get(k,'') for k in ['host_unix_ns','host_monotonic_ns','esp_ms','x_raw','y_raw','pressed']]+
                        [latest['angle_rad'],math.degrees(latest['angle_rad']),latest['angular_velocity_rad_s'],
                         latest['measurement_valid'],latest['clipped']])
                self.sequence += 1
            except (KeyError, ValueError):
                # Bad input must invalidate feedback, not refresh a prior angle.
                latest = None
                continue
        if self.output:self.output.flush()
        if latest is None or not rclpy.ok():
            return bool(chunk)
        msg = JoystickState()
        msg.header.frame_id = 'franka_base'
        msg.header.stamp.sec = latest.pop('stamp_ns') // 1_000_000_000
        # Recover ns without retimestamping a replayed/backlogged record.
        stamp = self.decoder.previous[0]
        msg.header.stamp.nanosec = stamp % 1_000_000_000
        msg.sequence = self.sequence
        for name, value in latest.items():
            setattr(msg, name, value)
        self.publisher.publish(msg)
        return bool(chunk)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output',type=Path,help='Continuous calibrated CSV alongside the raw recording')
    parser.add_argument('--center-x-raw',type=int)
    parser.add_argument('--center-y-raw',type=int)
    parser.add_argument('--topic', default='/joystick/shaft')
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros)
    node = None
    try:
        node = FeedbackNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            while node.read_latest():pass
            node.stream.close()
            if node.output:node.output.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
