#!/usr/bin/env python3
"""Run an operator-paced, paired physical experiment block; record each attempt."""
import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def check_controller(task,block):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    rclpy.init()
    node=rclpy.create_node('eit_block_config_check')
    try:
        name='/eit_joystick' if task=='joystick' else '/eit_ball_roll'
        client=node.create_client(GetParameters,name+'/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Launch the task controller first')
        names=['landing_offset_x_m','landing_offset_y_m','update_ball_attachment']
        future=client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        if not future.done() or future.result() is None:raise RuntimeError('Cannot read task configuration')
        values=future.result().values
        if any(v.type==0 for v in values):raise RuntimeError('Controller configuration is incomplete')
        expected={'center':0.,'x_minus':-.010,'x_plus':.010}.get(block,0.)
        if abs(values[0].double_value-expected)>1e-8 or abs(values[1].double_value)>1e-8:
            raise RuntimeError('Launch the matching landing-offset overlay before starting this block')
        if task=='ball' and values[2].bool_value:raise RuntimeError('Ball comparison requires update_ball_attachment=false')
        return {key:(v.bool_value if v.type==1 else v.double_value) for key,v in zip(names,values)}
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=['joystick','ball'],required=True)
    p.add_argument('--block',required=True,help='Joystick: center, x_minus, x_plus. Ball: undisturbed, push.')
    p.add_argument('--repeats',type=int,default=5)
    p.add_argument('--output',type=Path,default=Path.home()/'eit_trials')
    a=p.parse_args()
    blocks=['center','x_minus','x_plus'] if a.task=='joystick' else ['undisturbed','push']
    if a.block not in blocks or not 1<=a.repeats<=100:p.error('Choose a valid block and 1–100 repeats')
    checked=check_controller(a.task,a.block)
    script=Path(__file__).with_name('experiment.py')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder=a.output/a.task/(a.block+'_'+stamp)
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'block.json').write_text(json.dumps(vars(a)|{'output':str(a.output),'created_utc':stamp,'checked_controller_parameters':checked,
        'note':'Operator must launch the matching offset/config. Actual outcome comes from independent measurement, not controller done.'},indent=2)+'\n')
    print('This runs the physical robot. Use the matching launch/config; record independent object motion.')
    print('Each repeat tests both conditions. Order alternates across repeats. Logs:',folder)
    try:
        for repeat in range(1,a.repeats+1):
            order=['baseline','eit'] if repeat%2 else ['eit','baseline']
            for condition in order:
                print(f'\n{a.task} / {a.block} / repeat {repeat}/{a.repeats} / {condition}')
                if input('Clear the local ready-motion route. Type ready to move, or q to stop: ').strip().lower()!='ready':return
                subprocess.run([sys.executable,str(script),'--task',a.task,'--action','ready'],check=True)
                print('Reset the object to its marked start; leave the pad unloaded; start independent recording.')
                if a.task=='ball' and a.block=='push':
                    print('During rolling apply ONE push using the marked displacement/timing jig; controller gets no push flag.')
                else:print('Do not apply an external push during this trial.')
                if input('Type run to approach and manipulate, or q to stop: ').strip().lower()!='run':return
                destination=folder/f'repeat_{repeat:02d}'
                subprocess.run([sys.executable,str(script),'--task',a.task,'--action','trial',
                                '--condition',condition,'--output',str(destination)],check=True)
                outcome=input('Record independently observed result/recording ID (required): ').strip()
                if not outcome:raise RuntimeError('Outcome/recording ID required before proceeding')
                with (folder/'outcomes.jsonl').open('a') as stream:
                    stream.write(json.dumps({'repeat':repeat,'condition':condition,'observation':outcome,
                        'completed_utc':datetime.now(timezone.utc).isoformat()})+'\n')
        print('Block finished. CSV files record controller execution; evaluate object success independently.')
    except KeyboardInterrupt:
        print('\nBlock interrupted. Inspect the controller/robot state before continuing.')
        raise SystemExit(130)


if __name__=='__main__':main()
