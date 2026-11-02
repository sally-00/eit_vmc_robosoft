#!/usr/bin/env python3
"""Offline ball VMC replay from a paired run_block result; never accesses the robot.

When logged, the orange dot shows the EIT surface contact in the base frame.
Older CSVs lack contact X/Y and fall back to the controller's internal estimate.
Neither position is camera ground truth.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


BLUE = '#2563eb'
ORANGE = '#ea580c'
PURPLE = '#7c3aed'
GREEN = '#15803d'
RED = '#dc2626'
GRAY = '#64748b'


def read_trial(path, motion_ratio=.5):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError(f'Incomplete trace: {path}')
    def col(name):
        return np.array([float(row[name]) for row in rows])
    def flag(name):
        return np.array([row[name] == 'True' for row in rows])
    modes = np.array([row['mode'] for row in rows])
    engaged = np.flatnonzero(np.isin(modes, ['preload', 'interact']))
    ns = np.array([int(row['host_time_ns']) for row in rows], np.int64)
    start = int(engaged[0]) if len(engaged) else 0
    t = (ns - ns[start]) / 1e9
    axis = rows[start]['ball_axis']
    if axis not in ('x', 'y'):
        raise ValueError(f'Unexpected ball axis: {axis}')
    d = dict(path=str(path), rows=rows, t=t, modes=modes, axis=axis,
             control_law='direct VMC' if 'ball_normal_position_m' in rows[0] else 'legacy virtual mass',
             progress=col('progress_m'), goal=col('ball_goal_coordinate_m'),
             pad_x=col('pad_m_x'), pad_y=col('pad_m_y'), pad_z=col('pad_m_z'),
             pad_normal_x=col('pad_normal_base_x') if 'pad_normal_base_x' in rows[0] else None,
             pad_normal_y=col('pad_normal_base_y') if 'pad_normal_base_y' in rows[0] else None,
             ref_x=col('reference_m_x'), ref_y=col('reference_m_y'),
             force=col(f'coupling_force_N_{axis}'),
             contact=flag('sensor_contact'), candidate=flag('sensor_contact_candidate'),
             valid=flag('sensor_position_valid'), fresh=flag('sensor_fresh'),
             paused=flag('ball_contact_paused'),
             sensor_x=col('sensor_contact_base_m_x') if 'sensor_contact_base_m_x' in rows[0] else None,
             sensor_y=col('sensor_contact_base_m_y') if 'sensor_contact_base_m_y' in rows[0] else None,
             radius=col('sensor_predicted_radius_mm'), pattern=col('sensor_pattern_distance'),
             support=col('sensor_pattern_support_limit'),
             start=start, has_engagement=bool(len(engaged)), motion_ratio=motion_ratio,
             interaction=np.flatnonzero(modes == 'interact'),
             normal_position=col('ball_normal_position_m') if 'ball_normal_position_m' in rows[0] else None,
             normal_force=col('ball_normal_force_n') if 'ball_normal_force_n' in rows[0] else None,
             drive_reference=col('ball_drive_reference_m') if 'ball_drive_reference_m' in rows[0] else None,
             goal_reached=flag('ball_goal_reached') if 'ball_goal_reached' in rows[0] else None)
    pad_axis = d[f'pad_{axis}']
    d['robot_proxy'] = d['progress'][d['start']] + motion_ratio * (pad_axis - pad_axis[d['start']])
    if (d['sensor_x'] is None) != (d['sensor_y'] is None):
        raise ValueError(f'Incomplete sensor contact coordinates: {path}')
    if d['sensor_x'] is not None:
        d['sensor_xy_valid'] = (d['contact'] & d['valid'] & d['fresh'] &
                                np.isfinite(d['sensor_x']) & np.isfinite(d['sensor_y']))
    else:
        d['sensor_xy_valid'] = np.zeros(len(rows), dtype=bool)
    return d


def add_ee_origin(trial, block):
    """Reconstruct the EE origin only when the recorded offset is axial.

    The CSV has the full pad face and its normal, but not the full EE rotation.
    An axial EE-to-pad offset rotated about the pad normal is recoverable from
    those columns. Read the geometry used by this trial, not today's YAML.
    """
    trial['ee_x'] = trial['ee_y'] = None
    snapshot = (block/'reproducibility'/'snapshots'/
                (Path(trial['path']).stem+'.before_motion.json'))
    if not snapshot.is_file() or trial['pad_normal_x'] is None:
        return
    try:
        params = json.loads(snapshot.read_text())['nodes']['/eit_ball_roll']['parameters']
        offset = np.asarray(params['ee_to_pad_m']['value'], dtype=float)
        clocking = np.asarray(params['ee_R_pad_xyzw']['value'], dtype=float)
        if (offset.shape != (3,) or clocking.shape != (4,) or
                not np.all(np.isfinite(offset)) or not np.all(np.isfinite(clocking)) or
                not np.allclose(offset[:2], 0., atol=1e-9) or
                not np.allclose(clocking[:2], 0., atol=1e-9)):
            return
    except (KeyError, ValueError, TypeError, OSError, json.JSONDecodeError):
        return
    trial['ee_x'] = trial['pad_x'] - offset[2]*trial['pad_normal_x']
    trial['ee_y'] = trial['pad_y'] - offset[2]*trial['pad_normal_y']


def summarize(trial, initial):
    d = trial
    interact = d['interaction']
    end = int(interact[-1]) if len(interact) else len(d['rows'])-1
    start = d['start']
    pad = d[f'pad_{d["axis"]}']
    # Count pauses by sample duration; consecutive host samples are about 20 ms apart.
    dt = np.diff(d['t'], append=d['t'][-1])
    paused_seconds = float(np.sum(dt[interact] * d['paused'][interact]))
    return dict(source=d['path'], condition=Path(d['path']).name.split('_')[1],
                control_law=d['control_law'],
                interaction_duration_s=float(d['t'][end]-d['t'][interact[0]]) if len(interact) else 0.,
                pad_travel_mm=float((pad[end]-pad[start])*1000),
                controller_ball_start_m=float(d['progress'][start]),
                controller_ball_end_m=float(d['progress'][end]),
                sensor_xy_logged=d['sensor_x'] is not None,
                controller_ball_from_mark_mm=float((d['progress'][end]-initial)*1000),
                goal_m=float(d['goal'][end]),
                controller_goal_error_mm=float(abs(d['goal'][end]-d['progress'][end])*1000),
                baseline_motion_ratio=float(d['motion_ratio']),
                robot_motion_proxy_end_m=float(initial+d['motion_ratio']*(pad[end]-pad[start])),
                eit_contact_fraction_interact=float(np.mean(d['contact'][interact])) if len(interact) else None,
                eit_position_valid_fraction_interact=float(np.mean(d['valid'][interact])) if len(interact) else None,
                vmc_paused_fraction_interact=float(np.mean(d['paused'][interact])) if len(interact) else None,
                vmc_paused_seconds_interact=paused_seconds,
                controller_shift_during_preload_mm=float((d['progress'][interact[0]]-d['progress'][start])*1000) if len(interact) else None,
                controller_goal_reached=bool(d['goal_reached'][-1]) if d['goal_reached'] is not None else None,
                final_drive_reference_m=float(d['drive_reference'][end]) if d['drive_reference'] is not None else None,
                fault=next((row['fault'] for row in d['rows'] if row['fault'] != 'none'), None))


def sensor_caption(trial):
    return ('Orange dot = EIT surface contact when valid; purple ring = axis-only controller estimate.'
            if trial['sensor_x'] is not None else
            'Orange dot = controller estimate; EIT contact X/Y was not logged.')


def sensor_axis_from_mark(trial, initial):
    if trial['sensor_x'] is None:
        return None
    sensor = trial[f'sensor_{trial["axis"]}']
    return np.where(trial['sensor_xy_valid'], 1000*(sensor-initial), np.nan)


class PadScene:
    def __init__(self, ax, trial, initial_x, initial_y, goal, reported_start=None):
        self.ax, self.d = ax, trial
        self.x0, self.y0 = initial_x, initial_y
        ax.set_aspect('equal')
        signed_goal = (goal-initial_y)*1000 if trial['axis']=='y' else (goal-initial_x)*1000
        travel_extent=(0.,signed_goal,2.*signed_goal)
        if reported_start is not None:
            initial_axis=initial_y if trial['axis']=='y' else initial_x
            reported_offset=1000*(reported_start-initial_axis)
            travel_extent+=reported_offset,reported_offset+signed_goal
        axis_limits=(min(-58.,min(travel_extent)-30.),max(58.,max(travel_extent)+30.))
        ax.set_xlim(*(axis_limits if trial['axis']=='x' else (-58,58)))
        ax.set_ylim(*(axis_limits if trial['axis']=='y' else (-58,58)))
        ax.set_xlabel('Base X from start (mm)')
        ax.set_ylabel('Base Y from start (mm)')
        ax.grid(alpha=.15)
        ax.plot(0, 0, '+', color=GRAY, ms=10, mew=2)
        if trial['axis']=='y':
            ax.axhline(signed_goal, color=RED, ls='--', lw=1.5)
            if reported_start is not None and abs(reported_start-initial_y) > .001:
                ax.axhline(1000*(reported_start-initial_y), color=GREEN, ls=':', lw=1.5)
                ax.axhline(1000*(reported_start-initial_y)+signed_goal,
                           color=GREEN, ls='--', lw=1.5)
        else:
            ax.axvline(signed_goal, color=RED, ls='--', lw=1.5)
            if reported_start is not None and abs(reported_start-initial_x) > .001:
                ax.axvline(1000*(reported_start-initial_x), color=GREEN, ls=':', lw=1.5)
                ax.axvline(1000*(reported_start-initial_x)+signed_goal,
                           color=GREEN, ls='--', lw=1.5)
        ax.plot([], [], color=BLUE, lw=5, alpha=.45, label='robot pad path')
        self.pad_path, = ax.plot([], [], color=BLUE, lw=3, alpha=.55)
        self.pad = Circle((0, 0), 45, fc='#dbeafe', ec=BLUE, lw=2, alpha=.32)
        self.ball, = ax.plot([], [], 'o', color=ORANGE, ms=8)
        self.controller_ball, = ax.plot([], [], 'o', mfc='none', mec=PURPLE, mew=2, ms=10)
        ax.add_patch(self.pad)
        self.center, = ax.plot([], [], '+', color=BLUE, ms=13, mew=2.5)
        self.ee, = ax.plot([], [], 'D', color='#111827', ms=6)
        self.ref, = ax.plot([], [], 'x', color=PURPLE, ms=12, mew=2.5)
        self.spring, = ax.plot([], [], color=PURPLE, lw=2)
        self.label = ax.text(.02, .98, '', transform=ax.transAxes, va='top', fontsize=8,
                             bbox=dict(fc='white', ec='none', alpha=.9))
        ax.set_title(('Baseline' if 'baseline' in trial['path'] else 'EIT') +
                     (' | EIT contact and controller (top view)' if trial['sensor_x'] is not None
                      else ' | pad and controller ball (top view)'), weight='bold')

    def xy(self, x, y):
        return (1000*(x-self.x0), 1000*(y-self.y0))

    def update(self, at):
        d = self.d
        i = max(0, min(len(d['t'])-1, np.searchsorted(d['t'], at, side='right')-1))
        pad = self.xy(d['pad_x'][i], d['pad_y'][i])
        ref = self.xy(d['ref_x'][i], d['ref_y'][i])
        self.pad.center = pad
        self.center.set_data([pad[0]], [pad[1]])
        if d['ee_x'] is not None:
            ee = self.xy(d['ee_x'][i], d['ee_y'][i])
            self.ee.set_data([ee[0]], [ee[1]])
        if d['has_engagement'] and i >= d['start']:
            ball = self.xy(d['progress'][i], d['pad_y'][i]) if d['axis']=='x' else self.xy(d['pad_x'][i], d['progress'][i])
            if d['sensor_x'] is not None:
                self.controller_ball.set_data([ball[0]], [ball[1]])
                self.controller_ball.set_visible(True)
                if d['sensor_xy_valid'][i]:
                    contact = self.xy(d['sensor_x'][i], d['sensor_y'][i])
                    self.ball.set_data([contact[0]], [contact[1]])
                    self.ball.set_visible(True)
                else:
                    self.ball.set_visible(False)
            else:
                self.ball.set_data([ball[0]], [ball[1]])
                self.ball.set_visible(True)
                self.controller_ball.set_visible(False)
        else:
            self.ball.set_visible(False)
            self.controller_ball.set_visible(False)
        self.ref.set_data([ref[0]], [ref[1]])
        self.spring.set_data([pad[0], ref[0]], [pad[1], ref[1]])
        self.pad_path.set_data(1000*(d['pad_x'][:i+1]-self.x0), 1000*(d['pad_y'][:i+1]-self.y0))
        self.label.set_text(f'{d["modes"][i]} | {"paused" if d["paused"][i] else "active"}\n'
                            f'EIT: {"yes" if d["contact"][i] else "no"}, '
                            f'XY: {"valid" if d["valid"][i] else "invalid"}')


def render(block, output, fps=10, preview=False):
    files = {}
    for condition in ('baseline', 'eit'):
        matches = sorted(block.glob(f'repeat_*/ball_{condition}_*.csv'))
        if not matches:
            raise FileNotFoundError(f'No {condition} trial CSV in {block}')
        files[condition] = matches[-1]
    config = json.loads((block/'block.json').read_text())['checked_controller_parameters']
    motion_ratio = float(config.get('ball_baseline_motion_ratio', .5))
    trials = {name:read_trial(path,motion_ratio) for name,path in files.items()}
    for trial in trials.values():
        add_ee_origin(trial, block)
    axis = trials['eit']['axis']
    if trials['baseline']['axis'] != axis:
        raise ValueError('Paired trials use different ball axes')
    x0,y0 = float(config['initial_ball_x']), float(config['initial_ball_y'])
    initial = y0 if axis=='y' else x0
    goal = initial+float(config['roll_distance'])
    output.mkdir(parents=True, exist_ok=True)
    metrics = {name:summarize(d,initial) for name,d in trials.items()}
    for name in ('baseline','eit'):
        repeat=int(files[name].parent.name.split('_')[-1])
        camera=camera_record(block,name,repeat)
        displacement=camera.get('camera_ball_displacement_m')
        camera_start=camera.get('camera_ball_start_axis_m',initial)
        camera_final=camera.get('camera_ball_final_axis_m')
        if camera_final is None and displacement is not None:
            camera_final=camera_start+displacement
        metrics[name].update(camera_ball_displacement_m=displacement,
                             camera_ball_start_axis_m=camera_start,
                             camera_ball_final_axis_m=camera_final,
                             camera_ball_relative_goal_axis_m=camera.get('camera_ball_relative_goal_axis_m'))
    (output/'metrics.json').write_text(json.dumps(dict(block=str(block),axis=axis,
        initial_m=initial,goal_m=goal,notes=[
            'Orange dot is the EIT surface contact for new traces with valid X/Y; old traces show controller estimate.',
            'EIT contact location is not an independent camera measurement of the ball centre.',
            'Blue pad disc is the calculated pad face, not the robot EE origin; black diamond is the reconstructed EE origin when recorded geometry permits.',
            'Purple ring gives only the controller ball coordinate along the task axis; its other coordinate follows the pad for display.',
            f'Robot-only proxy is initial position plus {motion_ratio:g} times pad travel.',
            'Missing or invalid EIT positions are not interpolated in the replay.'],
        trials=metrics),indent=2)+'\n')

    fig = plt.figure(figsize=(16, 9), dpi=100, facecolor='white')
    gs = fig.add_gridspec(2, 3, width_ratios=[1.1,1.5,1.4],
                          left=.055,right=.975,top=.88,bottom=.14,wspace=.27,hspace=.4)
    scenes = [PadScene(fig.add_subplot(gs[j,0]), trials[name], x0,y0,goal,
                       metrics[name]['camera_ball_start_axis_m'])
              for j,name in enumerate(('baseline','eit'))]
    progress_ax = fig.add_subplot(gs[0,1])
    pad_ax = fig.add_subplot(gs[1,1],sharex=progress_ax)
    gate_ax = fig.add_subplot(gs[0,2],sharex=progress_ax)
    force_ax = fig.add_subplot(gs[1,2],sharex=progress_ax)
    colors = {'baseline':BLUE,'eit':ORANGE}
    for name,d in trials.items():
        c=colors[name]; t=d['t'];mask=t>=0
        progress_ax.plot(t[mask],1000*(d['progress'][mask]-initial),color=c,lw=2,label=f'{name} controller')
        sensed=sensor_axis_from_mark(d,initial)
        if sensed is not None:
            progress_ax.plot(t[mask],sensed[mask],color=c,ls='none',marker='.',ms=2,
                             alpha=.6,label=f'{name} EIT contact')
        if d['drive_reference'] is not None:
            progress_ax.plot(t[mask],1000*(d['drive_reference'][mask]-initial),
                             color=c,ls='--',lw=1.2,label=f'{name} moving spring goal')
        progress_ax.plot(t[mask],1000*motion_ratio*(d[f'pad_{axis}'][mask]-d[f'pad_{axis}'][d['start']]),
                         color=c,ls=':',lw=1.5,label=f'{name} robot {motion_ratio:g}× travel')
        camera_final=metrics[name]['camera_ball_final_axis_m']
        if camera_final is not None:
            progress_ax.axhline(1000*(camera_final-initial),color=c,ls='-.',lw=1.3,
                                label=f'{name} camera final')
        pad_ax.plot(t,1000*(d[f'pad_{axis}']-initial),color=c,lw=2,label=f'{name} pad face')
        if d['ee_x'] is not None:
            pad_ax.plot(t,1000*(d[f'ee_{axis}']-initial),color=c,ls=':',lw=1.4,
                        label=f'{name} EE origin')
        pad_ax.plot(t,1000*(d[f'ref_{axis}']-initial),color=c,ls='--',lw=1,
                    label=f'{name} goal-error/hold marker' if d['control_law']=='direct VMC'
                          else f'{name} position reference')
        force_ax.plot(t,d['force'],color=c,lw=1.5,label=f'{name} lateral VMC force')
    progress_ax.axhline(1000*(goal-initial),color=RED,ls='--',lw=1.5,label='goal')
    reported_starts={metrics[name]['camera_ball_start_axis_m'] for name in ('baseline','eit')}
    if len(reported_starts)==1:
        reported_start=reported_starts.pop()
        if abs(reported_start-initial)>.001:
            progress_ax.axhline(1000*(reported_start-initial),color=GREEN,ls=':',lw=1.3,
                                label='reported actual ball start')
            progress_ax.axhline(1000*(reported_start+float(config['roll_distance'])-initial),
                                color=GREEN,ls='--',lw=1.3,
                                label='10 cm from reported start')
    progress_ax.axhline(0,color=GRAY,lw=.7)
    progress_ax.set_ylabel(f'Controller ball {axis.upper()} from mark (mm)')
    progress_ax.set_title('Controller ball vs robot-motion proxy',weight='bold')
    progress_ax.legend(loc='best',fontsize=8,ncol=2)
    pad_ax.set_ylabel(f'Robot pad {axis.upper()} from mark (mm)')
    pad_ax.set_title('Measured pad and VMC spring target',weight='bold')
    pad_ax.legend(loc='best',fontsize=8,ncol=2)
    eit=trials['eit']
    gate_ax.step(eit['t'],eit['contact'].astype(float),where='post',color=GREEN,label='EIT contact')
    gate_ax.step(eit['t'],eit['valid'].astype(float),where='post',color=PURPLE,alpha=.7,label='EIT position valid')
    gate_ax.step(eit['t'],eit['paused'].astype(float),where='post',color=RED,alpha=.8,label='VMC paused')
    gate_ax.set(ylim=(-.1,1.12),ylabel='0 / 1')
    gate_ax.set_title('EIT classifier and controller gate',weight='bold')
    gate_ax.legend(loc='best',fontsize=8,ncol=2)
    force_ax.axhline(0,color=GRAY,lw=.7)
    force_ax.set_ylabel(f'VMC coupling force {axis.upper()} (N)')
    force_ax.set_title('Requested lateral spring force',weight='bold')
    force_ax.legend(loc='best',fontsize=8)
    tmax=max(d['t'][-1] for d in trials.values())
    tmin=min(d['t'][0] for d in trials.values())
    cursors=[]
    for ax in (progress_ax,pad_ax,gate_ax,force_ax):
        ax.set_xlim(tmin,tmax)
        ax.axvline(0,color=GRAY,ls=':',lw=1)
        ax.grid(alpha=.2)
        ax.set_xlabel('Seconds from preload start')
        cursors.append(ax.axvline(0,color='#111827',lw=1.3))
    fig.suptitle('Ball VMC replay | '+block.name+' | baseline and EIT',
                 x=.055,y=.973,ha='left',fontsize=17,weight='bold')
    fig.text(.055,.925,'Robot telemetry reconstruction. '+
             ('Orange dot = EIT surface contact when valid; purple ring = axis-only controller estimate.'
              if any(d['sensor_x'] is not None for d in trials.values()) else
              'Orange dot = controller estimate; EIT X/Y absent from these traces.')+
             ' Blue disc = calculated pad face; black diamond = robot EE origin.',
             fontsize=10,color='#334155')
    fig.text(.055,.065,'Configured ball start: '+f'{initial*1000:+.0f} mm; reported actual start: '
             +'/'.join(f'{metrics[name]["camera_ball_start_axis_m"]*1000:+.0f}' for name in ('baseline','eit'))
             +' mm (baseline/EIT). Green dotted line = actual start; green dashed = 10 cm from it.',fontsize=10,color=GRAY)
    fig.text(.055,.025,'No camera ball tracking. EIT dot is a surface contact, not the ball centre. Purple X is goal-error/hold telemetry.',
             fontsize=10,color=GRAY)

    def update(at):
        for scene in scenes:scene.update(at)
        for cursor in cursors:cursor.set_xdata([at,at])
        fig.canvas.draw()

    snapshot_at=min(8.,tmax)
    update(snapshot_at)
    fig.savefig(output/'ball_vmc_summary.png',dpi=150)
    if preview:
        plt.close(fig)
        return
    movie=output/'ball_vmc_replay.mp4'
    writer=cv2.VideoWriter(str(movie),cv2.VideoWriter_fourcc(*'avc1'),fps,(1600,900))
    if not writer.isOpened():
        plt.close(fig)
        raise RuntimeError('H.264 MP4 writer unavailable')
    try:
        for at in np.arange(tmin,tmax+.01,1./fps):
            update(at)
            rgba=np.asarray(fig.canvas.buffer_rgba())
            writer.write(cv2.cvtColor(rgba,cv2.COLOR_RGBA2BGR))
    finally:
        writer.release()
        plt.close(fig)
    cap=cv2.VideoCapture(str(movie))
    count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
    if count<2:
        raise RuntimeError(f'Video contains only {count} frames: {movie}')
    print(f'Saved {movie} ({count} frames), summary PNG, and metrics.json')


def camera_record(block, condition, repeat):
    path=block/'outcomes.jsonl'
    if not path.is_file():return {}
    for line in reversed(path.read_text().splitlines()):
        try:row=json.loads(line)
        except json.JSONDecodeError:continue
        if row.get('condition')==condition and row.get('repeat')==repeat:
            return row
    return {}


def render_single(block, path, output, fps=10):
    config=json.loads((block/'block.json').read_text())['checked_controller_parameters']
    motion_ratio=float(config.get('ball_baseline_motion_ratio',.5))
    d=read_trial(path,motion_ratio)
    add_ee_origin(d,block)
    x0,y0=float(config['initial_ball_x']),float(config['initial_ball_y'])
    axis=d['axis'];initial=y0 if axis=='y' else x0
    goal=initial+float(config['roll_distance'])
    condition='baseline' if 'baseline' in path.name else 'eit'
    repeat=int(path.parent.name.split('_')[-1])
    camera=camera_record(block,condition,repeat)
    measured=camera.get('camera_ball_displacement_m')
    camera_start=camera.get('camera_ball_start_axis_m',initial)
    camera_final=camera.get('camera_ball_final_axis_m')
    if camera_final is None and measured is not None:
        camera_final=camera_start+measured
    output.mkdir(parents=True,exist_ok=True)
    stem=path.stem+'_vmc'
    metrics=summarize(d,initial)
    metrics.update(camera_ball_displacement_m=measured,
                   camera_ball_start_axis_m=camera_start,
                   camera_ball_final_axis_m=camera_final,
                   camera_ball_relative_goal_axis_m=camera.get('camera_ball_relative_goal_axis_m'))
    (output/(stem+'_metrics.json')).write_text(json.dumps(metrics,indent=2)+'\n')

    fig=plt.figure(figsize=(16,9),dpi=100,facecolor='white')
    gs=fig.add_gridspec(2,3,width_ratios=[1.2,1.35,1.25],
                        left=.055,right=.97,top=.87,bottom=.10,wspace=.28,hspace=.35)
    scene=PadScene(fig.add_subplot(gs[:,0]),d,x0,y0,goal,camera_start)
    progress_ax=fig.add_subplot(gs[0,1]);pad_ax=fig.add_subplot(gs[1,1],sharex=progress_ax)
    gate_ax=fig.add_subplot(gs[0,2],sharex=progress_ax)
    force_ax=fig.add_subplot(gs[1,2],sharex=progress_ax)
    t=d['t'];mask=t>=0;pad=d[f'pad_{axis}']
    progress_ax.plot(t[mask],1000*(d['progress'][mask]-initial),color=ORANGE,lw=2,label='controller estimate')
    sensed=sensor_axis_from_mark(d,initial)
    if sensed is not None:
        progress_ax.plot(t[mask],sensed[mask],color=GREEN,ls='none',marker='.',ms=2,
                         alpha=.65,label='EIT contact estimate')
    if d['drive_reference'] is not None:
        progress_ax.plot(t[mask],1000*(d['drive_reference'][mask]-initial),
                         color=PURPLE,ls='--',lw=1.2,label='moving spring goal')
    progress_ax.plot(t[mask],1000*motion_ratio*(pad[mask]-pad[d['start']]),
                     color=BLUE,ls=':',lw=1.6,label=f'robot {motion_ratio:g}×-travel proxy')
    progress_ax.axhline(1000*(goal-initial),color=RED,ls='--',label='10 cm goal')
    if camera_final is not None:
        progress_ax.axhline(1000*(camera_final-initial),color='#0f766e',ls='-.',
                            label='camera final')
    if abs(camera_start-initial)>.001:
        progress_ax.axhline(1000*(camera_start-initial),color=GREEN,ls=':',
                            label='reported actual ball start')
        progress_ax.axhline(1000*(camera_start+float(config['roll_distance'])-initial),
                            color=GREEN,ls='--',label='10 cm from reported start')
    progress_ax.set(ylabel=f'Ball {axis.upper()} from mark (mm)',title='Controller estimate vs goal')
    progress_ax.legend(fontsize=8,loc='best')
    pad_ax.plot(t,1000*(pad-initial),color=BLUE,lw=2,label='calculated pad face')
    if d['ee_x'] is not None:
        pad_ax.plot(t,1000*(d[f'ee_{axis}']-initial),color='#111827',ls=':',lw=1.4,
                    label='robot EE origin')
    pad_ax.plot(t,1000*(d[f'ref_{axis}']-initial),color=PURPLE,ls='--',
                label='goal-error/hold marker' if d['control_law']=='direct VMC'
                      else 'position reference')
    pad_ax.set(ylabel=f'Pad {axis.upper()} from mark (mm)',title='Robot pad motion')
    pad_ax.legend(fontsize=8,loc='best')
    gate_ax.step(t,d['contact'].astype(float),where='post',color=GREEN,label='EIT contact')
    gate_ax.step(t,d['valid'].astype(float),where='post',color=PURPLE,label='position valid')
    gate_ax.step(t,d['paused'].astype(float),where='post',color=RED,label='lateral paused')
    gate_ax.set(ylim=(-.1,1.12),ylabel='0 / 1',title='Contact and drive state')
    gate_ax.legend(fontsize=8,loc='best')
    force_ax.plot(t,d['force'],color=BLUE,label='lateral VMC force')
    if d['normal_force'] is not None:
        force_ax.plot(t,d['normal_force'],color=GREEN,label='normal spring force')
    force_ax.set(ylabel='Requested force (N)',title='VMC force and normal carriage')
    if d['normal_position'] is not None:
        normal_ax=force_ax.twinx()
        normal_ax.plot(t,1000*d['normal_position'],color=PURPLE,alpha=.65,label='normal carriage Z')
        normal_ax.set_ylabel('Carriage Z (mm)')
    force_ax.legend(fontsize=8,loc='best')
    cursors=[]
    for ax in (progress_ax,pad_ax,gate_ax,force_ax):
        ax.set_xlim(t[0],t[-1]);ax.grid(alpha=.2);ax.set_xlabel('Seconds from contact engagement')
        cursors.append(ax.axvline(0,color='#111827',lw=1.3))
    fig.suptitle(f'Ball VMC replay | {block.name} | repeat {repeat} | {condition}',
                 x=.055,y=.97,ha='left',fontsize=17,weight='bold')
    camera_text=(f'Camera: start {1000*camera_start:+.0f}, final {1000*camera_final:+.0f} mm '
                 f'(travel {1000*measured:+.0f} mm)'
                 if measured is not None else 'Camera-measured travel: pending')
    fig.text(.055,.92,camera_text+' | '+sensor_caption(d)+' Neither is camera tracking.',
             fontsize=11,color='#334155')
    fig.text(.055,.02,'Blue disc is pad face, black diamond is robot EE origin; EIT dot is surface contact, not ball centre.',
             fontsize=10,color=GRAY)
    def update(at):
        scene.update(at)
        for cursor in cursors:cursor.set_xdata([at,at])
        fig.canvas.draw()
    update(min(8.,t[-1]))
    fig.savefig(output/(stem+'.png'),dpi=150)
    movie=output/(stem+'.mp4')
    writer=cv2.VideoWriter(str(movie),cv2.VideoWriter_fourcc(*'avc1'),fps,(1600,900))
    if not writer.isOpened():
        plt.close(fig);raise RuntimeError('H.264 MP4 writer unavailable')
    try:
        for at in np.arange(t[0],t[-1]+.01,1./fps):
            update(at)
            writer.write(cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()),cv2.COLOR_RGBA2BGR))
    finally:
        writer.release();plt.close(fig)
    cap=cv2.VideoCapture(str(movie));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
    if count<2:raise RuntimeError(f'Video contains only {count} frames: {movie}')
    print(f'Saved {movie} ({count} frames)')


def render_all(block,output,fps=10):
    paths=sorted(block.glob('repeat_*/ball_baseline_*.csv'))+sorted(block.glob('repeat_*/ball_eit_*.csv'))
    errors=[]
    for path in paths:
        try:render_single(block,path,output/path.parent.name,fps)
        except Exception as exc:errors.append(f'{path.name}: {exc}')
    if any('baseline' in p.name for p in paths) and any('eit' in p.name for p in paths):
        try:render(block,output,fps)
        except Exception as exc:errors.append(f'paired: {exc}')
    if errors:raise RuntimeError('; '.join(errors))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('block',type=Path,help='Ball run_block result folder')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fps',type=int,default=10)
    parser.add_argument('--preview',action='store_true',help='Render PNG and metrics only')
    parser.add_argument('--all',action='store_true',help='Render every individual ball trial and a paired replay')
    args=parser.parse_args()
    if args.all:render_all(args.block,args.output,args.fps)
    else:render(args.block,args.output,args.fps,args.preview)
