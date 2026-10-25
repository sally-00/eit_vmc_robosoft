#!/usr/bin/env python3
"""Run paired physical trials automatically for joystick or with prompts for ball."""
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
    command=[str(joystick_recorder_script()),'--no-plot','--output',str(csv_path)]
    if port:command.extend(['--port',port])
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


def joystick_is_centered(path,center_x,center_y,tolerance):
    with path.open(newline='') as stream:
        rows=deque(csv.DictReader(stream),maxlen=50)
    samples=[]
    for row in rows:
        try:samples.append((int(row['host_unix_ns']),int(row['x_raw']),int(row['y_raw'])))
        except (KeyError,TypeError,ValueError):continue
    if len(samples)<30 or time.time_ns()-samples[-1][0]>500_000_000:return False
    xs=[row[1] for row in samples];ys=[row[2] for row in samples]
    x=statistics.median(xs);y=statistics.median(ys)
    return (abs(x-center_x)<=tolerance and abs(y-center_y)<=tolerance and
            max(abs(value-x) for value in xs)<=60 and
            max(abs(value-y) for value in ys)<=60)


def wait_for_joystick_center(csv_path,recorder,recorder_log,center_x,center_y,tolerance):
    deadline=time.monotonic()+20.
    waiting=False
    while True:
        check_joystick_recorder(recorder,recorder_log)
        if joystick_is_centered(csv_path,center_x,center_y,tolerance):return
        if time.monotonic()>=deadline:
            raise RuntimeError('Joystick did not return to its calibrated centre within 20 s; '
                               f'check the fixture and ESP32 recording {csv_path}')
        if not waiting:
            print('Waiting for the joystick to return upright before automatic motion...',flush=True)
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
    result=subprocess.run([sys.executable,str(script),str(folder)],
                          capture_output=True,text=True)
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
        names=['landing_offset_x_m','landing_offset_y_m','update_ball_attachment']
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
        return {key:(v.bool_value if v.type==1 else v.double_value) for key,v in zip(names,values)}
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=['joystick','ball'],required=True)
    p.add_argument('--block',required=True,help='Joystick: center, x_minus, x_plus. Ball: undisturbed, push.')
    p.add_argument('--repeats',type=int,default=5)
    p.add_argument('--output',type=Path,default=Path.home()/'eit_trials')
    p.add_argument('--config-dir',type=Path,help='Directory containing the same YAML files used to launch the controller; defaults to EIT_CONFIG_DIR or this checkout')
    p.add_argument('--joystick-port',help='ESP32 serial port; defaults to CH340 auto-detection')
    p.add_argument('--manual',action='store_true',help='Keep ready/run keyboard prompts for joystick blocks')
    p.add_argument('--center-x-raw',type=int,help='Override the calibrated upright ESP32 X reading')
    p.add_argument('--center-y-raw',type=int,help='Override the calibrated upright ESP32 Y reading')
    p.add_argument('--center-tolerance-raw',type=int,default=80,help='Allowed ADC difference from upright on each axis')
    a=p.parse_args()
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
    blocks=['center','x_minus','x_plus'] if a.task=='joystick' else ['undisturbed','push']
    if a.block not in blocks or not 1<=a.repeats<=100:p.error('Choose a valid block and 1–100 repeats')
    if a.center_tolerance_raw<=0 or not all(0<=value<=4095 for value in
            (a.center_x_raw,a.center_y_raw)):p.error('Choose valid ESP32 centre readings and tolerance')
    auto=a.task=='joystick' and not a.manual
    checked=check_controller(a.task,a.block,a.config_dir)
    script=Path(__file__).with_name('experiment.py')
    created=datetime.now(timezone.utc)
    stamp=created.astimezone().strftime('%Y-%m-%d_%H-%M-%S_%f_%z')
    folder=a.output/a.task/(a.block+'_'+stamp)
    folder.mkdir(parents=True,exist_ok=False)
    if calibration is not None:
        shutil.copyfile(calibration_path,folder/'joystick_adc_calibration.json')
    (folder/'block.json').write_text(json.dumps(vars(a)|{'output':str(a.output),'config_dir':str(config_directory(a.config_dir)),'created_utc':created.isoformat(),'automatic_joystick':auto,'checked_controller_parameters':checked,
        'joystick_recorder_csv':str(folder/'joystick_esp32.csv') if a.task=='joystick' else None,
        'note':'Operator must launch the matching offset/config. Actual outcome comes from independent measurement, not controller done.'},indent=2)+'\n')
    print('This runs the physical robot. Use the matching launch/config; record independent object motion.')
    print('Each repeat tests both conditions. Order alternates across repeats. Logs:',folder)
    if auto:print('Automatic joystick sequence: ready, approach, manipulate and retract for each condition; no keyboard prompts.',flush=True)
    recorder=None
    recorder_log=None
    try:
        if a.task=='joystick':
            print('Starting ESP32 recorder before any robot motion...',flush=True)
            recorder,recorder_csv,recorder_log=start_joystick_recorder(folder,a.joystick_port)
            print('ESP32 samples:',recorder_csv,flush=True)
        for repeat in range(1,a.repeats+1):
            order=['baseline','eit'] if repeat%2 else ['eit','baseline']
            for condition in order:
                print(f'\n{a.task} / {a.block} / repeat {repeat}/{a.repeats} / {condition}')
                destination=folder/f'repeat_{repeat:02d}'
                if recorder:check_joystick_recorder(recorder,recorder_log)
                if auto:
                    wait_for_joystick_center(recorder_csv,recorder,recorder_log,
                                             a.center_x_raw,a.center_y_raw,a.center_tolerance_raw)
                elif require_command('Clear the local ready-motion route; type ready or r and press Enter (q stops): ',('ready','r')) is None:return
                subprocess.run([sys.executable,str(script),'--task',a.task,'--action','ready',
                                '--output',str(destination)],check=True)
                if recorder:check_joystick_recorder(recorder,recorder_log)
                if auto:
                    wait_for_joystick_center(recorder_csv,recorder,recorder_log,
                                             a.center_x_raw,a.center_y_raw,a.center_tolerance_raw)
                elif a.task=='joystick':
                    print('Reset joystick upright; leave pad unloaded. ESP32 recording:',recorder_csv)
                else:
                    print('Reset the object to its marked start; leave the pad unloaded; start independent camera recording.')
                if a.task=='ball' and a.block=='push':
                    print('During rolling apply ONE push using the marked displacement/timing jig; controller gets no push flag.')
                else:print('Do not apply an external push during this trial.')
                if not auto and require_command('Type run or r and press Enter to approach and manipulate (q stops): ',('run','r')) is None:return
                if recorder:check_joystick_recorder(recorder,recorder_log)
                try:
                    subprocess.run([sys.executable,str(script),'--task',a.task,'--action','trial',
                                    '--condition',condition,'--output',str(destination)],check=True)
                finally:
                    if recorder and not auto:plot_joystick_block(folder)
                if recorder:check_joystick_recorder(recorder,recorder_log)
                if a.task=='joystick':
                    outcome=None
                    status='pending_esp32_review'
                    if not auto:print('ESP32 result saved for later review; continue with the next ready prompt.')
                else:
                    print('INPUT REQUIRED: Enter what the camera showed, for example '
                          '"final ball X=0.47 m; reached goal=yes; contact lost=no".',flush=True)
                    while True:
                        outcome=input('Ball observation (free text; press Enter to save, or type q to stop): ').strip()
                        if outcome.lower()=='q':return
                        if outcome:break
                        print('INPUT REQUIRED: Describe the camera result, such as final ball X and whether it reached the goal, then press Enter; or type q to stop.',flush=True)
                    status='observed'
                with (folder/'outcomes.jsonl').open('a') as stream:
                    stream.write(json.dumps({'repeat':repeat,'condition':condition,'observation':outcome,
                        'observation_status':status,
                        'joystick_recorder_csv':str(recorder_csv) if recorder else None,
                        'completed_utc':datetime.now(timezone.utc).isoformat()})+'\n')
        print('Block finished. CSV files record controller execution; evaluate object success independently.')
    except KeyboardInterrupt:
        print('\nBlock interrupted. Inspect the controller/robot state before continuing.')
        raise SystemExit(130)
    finally:
        if auto and recorder:stop_controller(script,a.task)
        if recorder:
            stop_joystick_recorder(recorder)
            print('ESP32 recording saved:',recorder_csv,flush=True)
            plot_joystick_block(folder)


if __name__=='__main__':
    try:main()
    except RuntimeError as exc:
        print('Block stopped: '+str(exc),file=sys.stderr)
        raise SystemExit(1)
