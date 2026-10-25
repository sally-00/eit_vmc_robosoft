#!/usr/bin/env python3
"""Show the live EIT contact estimate on the contacting face of the pad."""
import argparse
import hashlib
import json
import math
import time
import tkinter as tk
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from franka_eit_vmc.msg import EitContact
from franka_eit_tools.model_selection import select_model


def model_geometry(path):
    with np.load(path, allow_pickle=False) as data:
        metadata = json.loads(str(data['metadata']))
    radius_m = float(metadata['radius_mm']) / 1000.
    if not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError('Model has an invalid pad radius')
    return radius_m, metadata['numbering']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', help='Override the newest model in contact_position_runs')
    parser.add_argument('--timeout', type=float, default=0.30,
                        help='Seconds without a message before showing STALE (default: 0.30)')
    args = parser.parse_args()
    model_path = select_model(args.model)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive')
    radius_m, numbering = model_geometry(model_path)
    model_sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
    print(f'EIT model: {model_path} (SHA-256 {model_sha256})', flush=True)

    rclpy.init()
    node = rclpy.create_node('eit_pad_viewer')
    root = tk.Tk()
    root.title('EIT pad contact')
    canvas = tk.Canvas(root, width=440, height=560, bg='white', highlightthickness=0)
    canvas.pack()
    latest = [None, 0.]
    client = node.create_client(SetParameters, '/eit_contact/set_parameters')
    slider_state = {'initialized': False, 'suppress': False, 'timer': None,
                    'future': None, 'target': None, 'awaiting_echo': False,
                    'sent_at': 0.}
    slider_status = tk.StringVar(value='Waiting for EIT threshold data')

    def send_threshold():
        slider_state['timer'] = None
        if slider_state['future'] is not None:
            slider_state['timer'] = root.after(100, send_threshold)
            return
        if not client.service_is_ready():
            slider_status.set('EIT node unavailable; threshold unchanged')
            return
        value = float(threshold_slider.get())
        request = SetParameters.Request(parameters=[Parameter(
            name='contact_threshold',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=value))])
        slider_state['target'] = value
        slider_state['future'] = client.call_async(request)
        slider_status.set(f'Applying {value:.6f}...')

    def on_slider_change(_value):
        if slider_state['suppress'] or not slider_state['initialized']:
            return
        if slider_state['timer'] is not None:
            root.after_cancel(slider_state['timer'])
        slider_state['timer'] = root.after(180, send_threshold)

    tk.Label(root, text='Contact threshold (dimensionless)').pack()
    threshold_slider = tk.Scale(root, from_=0., to=.01, resolution=.00001,
                                orient=tk.HORIZONTAL, length=400, showvalue=True,
                                command=on_slider_change)
    threshold_slider.pack()
    tk.Label(root, textvariable=slider_status).pack()

    def on_message(message):
        latest[0] = message
        latest[1] = time.monotonic()
        value = float(message.contact_threshold)
        if not math.isfinite(value) or value < 0:
            return
        upper = max(.01, value * 1.25,
                    float(message.signal_strength) * 2 if math.isfinite(message.signal_strength) else 0.)
        if upper > float(threshold_slider.cget('to')):
            slider_state['suppress'] = True
            threshold_slider.configure(to=upper)
            slider_state['suppress'] = False
        if slider_state['awaiting_echo']:
            if abs(value - slider_state['target']) <= 1e-5:
                slider_state['awaiting_echo'] = False
                slider_status.set(f'Active threshold {value:.6f}')
            elif time.monotonic() - slider_state['sent_at'] < 2.:
                return
            else:
                slider_state['awaiting_echo'] = False
        if slider_state['future'] is None and slider_state['timer'] is None:
            slider_state['suppress'] = True
            threshold_slider.set(value)
            slider_state['suppress'] = False
            slider_state['initialized'] = True

    def check_threshold_reply():
        future = slider_state['future']
        if future is None or not future.done():
            return
        slider_state['future'] = None
        try:
            reply = future.result()
            result = reply.results[0]
            if result.successful:
                slider_state['awaiting_echo'] = True
                slider_state['sent_at'] = time.monotonic()
                slider_status.set(f'Applied {slider_state["target"]:.6f}; waiting for sensor echo')
            else:
                slider_status.set('Threshold rejected: ' + result.reason)
        except Exception as exc:
            slider_status.set('Threshold update failed: ' + str(exc))
        if slider_state['timer'] is not None:
            root.after_cancel(slider_state['timer'])
            slider_state['timer'] = root.after(0, send_threshold)

    node.create_subscription(EitContact, '/eit/contact', on_message, 10)

    def redraw():
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.)
        check_threshold_reply()
        canvas.delete('all')
        cx, cy, radius_px = 220, 245, 175
        canvas.create_oval(cx-radius_px, cy-radius_px, cx+radius_px, cy+radius_px,
                           fill='#f6f8fb', outline='#374151', width=2)
        canvas.create_line(cx-radius_px, cy, cx+radius_px, cy, fill='#d1d5db')
        canvas.create_line(cx, cy-radius_px, cx, cy+radius_px, fill='#d1d5db')
        canvas.create_text(cx, cy-radius_px-24, text='Pad contacting-face view',
                           font=('Sans', 14, 'bold'))
        canvas.create_text(cx+radius_px+15, cy, text='+X', anchor='w')
        canvas.create_text(cx, cy-radius_px-7, text='+Y', anchor='s')
        for number in (1, 3, 5, 7):
            angle = math.radians((-90+45*(number-1)) if numbering == 'ccw'
                                 else (90-45*(number-1)))
            canvas.create_text(cx+(radius_px-18)*math.cos(angle),
                               cy-(radius_px-18)*math.sin(angle), text=str(number),
                               fill='#6b7280')

        message, received = latest
        sample_age = (time.time_ns() - (message.header.stamp.sec*1_000_000_000
                       + message.header.stamp.nanosec))/1e9 if message else float('inf')
        if message is None or max(time.monotonic()-received, sample_age) > args.timeout:
            status, color, detail = 'STALE / NO DATA', '#b45309', 'No fresh /eit/contact message'
        elif message.model_sha256 != model_sha256:
            status, color, detail = 'MODEL MISMATCH', '#b45309', 'Restart viewer with the node model'
        elif not message.measurement_valid or not message.model_is_physical:
            status, color, detail = 'INVALID / CALIBRATING', '#b45309', 'Check baseline and sensor stream'
        elif not message.contact:
            status, color, detail = 'NO CONTACT', '#4b5563', 'Pad unloaded'
        elif not message.position_valid:
            status, color, detail = 'CONTACT, POSITION INVALID', '#b45309', 'No usable location'
        else:
            status, color = 'CONTACT', '#047857'
            detail = f'X {message.x_m*1000:+.1f} mm    Y {message.y_m*1000:+.1f} mm'
            px = cx + radius_px*message.x_m/radius_m
            py = cy - radius_px*message.y_m/radius_m
            if math.hypot(message.x_m, message.y_m) <= radius_m:
                canvas.create_oval(px-9, py-9, px+9, py+9, fill=color, outline='white', width=2)
        canvas.create_text(cx, 453, text=status, fill=color, font=('Sans', 17, 'bold'))
        canvas.create_text(cx, 480, text=detail, fill='#374151', font=('Sans', 11))
        if message is not None and max(time.monotonic()-received, sample_age) <= args.timeout \
                and message.measurement_valid:
            signal_text = f'Signal strength {message.signal_strength:.6f}'
        else:
            signal_text = 'Signal strength --'
        canvas.create_text(cx, 510, text=signal_text, fill='#374151',
                           font=('Sans', 12, 'bold'))
        threshold_text = (f'Contact threshold {message.contact_threshold:.6f}'
                          if message is not None else 'Contact threshold --')
        canvas.create_text(cx, 536, text=threshold_text,
                           fill='#6b7280', font=('Sans', 10))
        root.after(50, redraw)

    try:
        redraw()
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
