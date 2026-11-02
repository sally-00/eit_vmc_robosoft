#!/usr/bin/env python3
"""Plot controller traces beside the independent ESP32 recording in a block folder."""
import argparse
import csv
import json
import sys
import textwrap
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


CONDITIONS = ('baseline', 'eit', 'kinematic')
COLORS = {'baseline': '#1764ab', 'eit': '#d35400', 'kinematic': '#15936e'}
LABELS = {'baseline': 'without EIT', 'eit': 'with EIT', 'kinematic': 'kinematic tip reference'}


def trace_condition(path):
    return next((name for name in CONDITIONS if path.name.startswith(f'joystick_{name}_')), 'baseline')


def conditions_in_folder(folder):
    path = folder/'block.json'
    metadata = json.loads(path.read_text()) if path.is_file() else {}
    names = metadata.get('conditions', ['baseline', 'eit'])
    present = [name for name in CONDITIONS if any(folder.glob(f'repeat_*/joystick_{name}_*.csv'))]
    return [name for name in CONDITIONS if name in names or name in present]


def read_rows(path):
    with path.open(newline='') as stream:
        return [row for row in csv.DictReader(stream)
                if None not in row and all(value is not None for value in row.values())]


def read_esp(path):
    keys = ('host_unix_ns', 'x_raw', 'y_raw')
    records = []
    if path.is_file():
        for row in read_rows(path):
            try: records.append(tuple(int(row[key]) for key in keys))
            except (KeyError, ValueError): continue
    return {key: np.array([row[index] for row in records], dtype=np.int64)
            for index, key in enumerate(keys)}


def load_calibration(folder):
    path = folder / 'joystick_adc_calibration.json'
    return json.loads(path.read_text()) if path.is_file() else None


def angle_from_raw(values, specification, center_offset=0., limit_deg=35.):
    raw = np.asarray(values, dtype=float) - center_offset
    center = float(specification['center_raw'])
    plus = float(specification['plus_limit_raw'])
    minus = float(specification['minus_limit_raw'])
    if not 0 <= plus < center < minus <= 4095:
        raise ValueError('Invalid joystick ADC calibration endpoints')
    angle = np.where(raw <= center,
                     limit_deg * (center - raw) / (center - plus),
                     -limit_deg * (raw - center) / (minus - center))
    return np.clip(angle, -limit_deg, limit_deg)


def measured_angles(x, y, calibration, observed_center_x=None, observed_center_y=None):
    mapping = calibration['mapping']
    limit = float(calibration['nominal_limit_deg'])
    base_x = mapping['base_x_deg']
    base_y = mapping['base_y_deg']
    offset_x = (observed_center_x - float(base_y['center_raw'])
                if observed_center_x is not None and np.isfinite(observed_center_x) else 0.)
    offset_y = (observed_center_y - float(base_x['center_raw'])
                if observed_center_y is not None and np.isfinite(observed_center_y) else 0.)
    return (angle_from_raw(y, base_x, offset_y, limit),
            angle_from_raw(x, base_y, offset_x, limit))


def column(rows, key):
    return np.array([float(row[key]) for row in rows])


def selected_axis(rows):
    # Old traces predate axis selection and always commanded Base X tilt.
    return rows[0].get('joystick_axis', 'x')


def command_column(rows):
    return column(rows, 'theta_d_y_rad' if selected_axis(rows) == 'y' else 'theta_d_rad')


def trace_status(rows):
    if not rows:
        return 'no samples saved'
    fault = next((row.get('fault') for row in rows if row.get('fault', 'none') != 'none'), None)
    if fault:
        return 'FAULT: ' + fault
    if rows[-1].get('mode') != 'done':
        return 'INCOMPLETE: ended in ' + rows[-1].get('mode', 'unknown')
    if rows[-1].get('trial_contact_released') == 'True':
        return 'failed: joystick slip' if rows[-1].get('joystick_slipped') == 'True' else 'unsuccessful: contact release'
    if rows[-1].get('retract_incomplete') == 'True':
        return 'manipulation finished; retract incomplete'
    return 'completed'


def normal_columns(rows, prefix):
    return np.column_stack([column(rows, f'{prefix}_{axis}') for axis in 'xyz'])


def palm_orientation(rows):
    if not {'pad_normal_base_x', 'desired_pad_normal_base_x',
            'joystick_shaft_angle_rad', 'joystick_shaft_angle_y_rad'} <= rows[0].keys():
        return None
    actual = normal_columns(rows, 'pad_normal_base')
    desired = normal_columns(rows, 'desired_pad_normal_base')
    shaft = column(rows, 'joystick_shaft_angle_rad')
    shaft_y = column(rows, 'joystick_shaft_angle_y_rad')
    valid = np.array([row['joystick_shaft_fresh'] == 'True' for row in rows])
    shaft = np.where(valid, shaft, np.nan)
    shaft_y = np.where(valid, shaft_y, np.nan)
    axis = np.column_stack((np.tan(shaft), np.tan(shaft_y), np.ones(len(shaft))))
    axis /= np.linalg.norm(axis, axis=1)[:, None]
    index = 1 if selected_axis(rows) == 'y' else 0
    actual_tilt = np.rad2deg(np.arctan2(-actual[:, index], -actual[:, 2]))
    desired_tilt = np.rad2deg(np.arctan2(-desired[:, index], -desired[:, 2]))
    shaft_tilt = np.rad2deg(shaft_y if index else shaft)
    perpendicular_error = np.rad2deg(np.arccos(np.clip(-np.sum(actual * axis, axis=1), -1., 1.)))
    target_error = np.rad2deg(np.arccos(np.clip(np.sum(actual * desired, axis=1), -1., 1.)))
    return desired_tilt, actual_tilt, shaft_tilt, perpendicular_error, target_error


def sensor_window(esp, origin_ns, low_s, high_s):
    t = (esp['host_unix_ns'] - origin_ns) / 1e9
    selected = (t >= low_s) & (t <= high_s)
    return t[selected], esp['x_raw'][selected], esp['y_raw'][selected]


def sensor_center(esp, origin_ns):
    t = (esp['host_unix_ns'] - origin_ns) / 1e9
    selected = (t >= -2.) & (t < 0.)
    if not selected.any():
        return float('nan'), float('nan')
    return float(np.median(esp['x_raw'][selected])), float(np.median(esp['y_raw'][selected]))


def smooth_sensor(t, values, width_s=.1):
    if not len(t):
        return np.array([]), np.array([])
    bins = np.floor(t / width_s).astype(int)
    unique, start = np.unique(bins, return_index=True)
    end = np.r_[start[1:], len(bins)]
    return (unique + .5) * width_s, np.array(
        [np.median(values[a:b]) for a, b in zip(start, end)])


def trial_origin(rows):
    return int(rows[0]['host_time_ns'])


def interact_origin(rows):
    return next((int(row['host_time_ns']) for row in rows if row['mode'] == 'interact'), None)


def draw_trial(trace, esp, calibration=None):
    rows = read_rows(trace)
    if not rows:
        return None
    tilt_axis = selected_axis(rows)
    origin = trial_origin(rows)
    time = (np.array([int(row['host_time_ns']) for row in rows]) - origin) / 1e9
    end = max(1., time[-1])
    condition = trace_condition(trace)
    color = COLORS[condition]
    sensor_logged = 'sensor_contact' in rows[0]
    pattern_logged = 'sensor_pattern_distance' in rows[0]
    panels = 7 if pattern_logged else 6 if sensor_logged else 5
    fig, axes = plt.subplots(panels, 1, figsize=(11, 15 if pattern_logged else 13 if sensor_logged else 11),
                            sharex=True)
    axes[0].plot(time, np.rad2deg(command_column(rows)), color=color,
                 label=f'Desired Base {tilt_axis.upper()} angle')
    axes[0].set_ylabel('Virtual angle (deg)')
    point = column(rows, f'point_m_{tilt_axis}')
    reference = column(rows, f'reference_m_{tilt_axis}')
    axes[1].plot(time, (reference - point[0]) * 1000, '--', color=color, label='Reference')
    axes[1].plot(time, (point - point[0]) * 1000, color=color, label='Measured point')
    axes[1].set_ylabel(f'{tilt_axis.upper()} displacement (mm)')
    axes[1].legend(loc='upper right')
    axes[2].plot(time, column(rows, 'requested_force_N_x'), color=color, label='X')
    axes[2].plot(time, column(rows, 'requested_force_N_y'), color='#15936e', label='Y')
    axes[2].plot(time, column(rows, 'requested_force_N_z'), color='#555555', label='Z')
    axes[2].set_ylabel('Requested force (N)')
    axes[2].legend(loc='upper right')
    for axis, color in (('x', '#1764ab'), ('y', '#15936e'), ('z', '#a35050')):
        axes[3].plot(time, column(rows, f'external_force_N_{axis}'), color=color,
                     label=axis.upper())
    axes[3].set_ylabel('External force (N)')
    axes[3].legend(loc='upper right', ncol=3)
    sensor_t, x, y = sensor_window(esp, origin, -2., end + 2.)
    center_x, center_y = sensor_center(esp, origin)
    if calibration:
        angle_x, angle_y = measured_angles(x, y, calibration)
        for values, label, style in ((angle_x, 'Base X angle', '-'),
                                     (angle_y, 'Base Y angle', '--')):
            plot_t, plot_y = smooth_sensor(sensor_t, values)
            axes[4].plot(plot_t, plot_y, style, label=label)
            if label == f'Base {tilt_axis.upper()} angle':
                axes[0].plot(plot_t, plot_y, color='#15936e', alpha=.8,
                             label=f'Measured Base {tilt_axis.upper()} angle')
        axes[0].legend(loc='best', fontsize=8)
        axes[4].set_ylabel('Joystick angle (deg)')
        axes[4].plot(time, np.rad2deg(column(rows, 'theta_d_rad')), '--',
                     color='#555555', label='Desired Base X angle')
        if 'theta_d_y_rad' in rows[0]:
            axes[4].plot(time, np.rad2deg(column(rows, 'theta_d_y_rad')), ':',
                         color='#a35050', label='Desired Base Y angle')
    else:
        for values, center, label, style in ((x, center_x, 'ESP32 X', '--'),
                                             (y, center_y, 'ESP32 Y', '-')):
            if np.isfinite(center):
                plot_t, plot_y = smooth_sensor(sensor_t, values - center)
                axes[4].plot(plot_t, plot_y, style, label=label)
        axes[4].set_ylabel('ADC change (counts)')
    axes[4].legend(loc='best')
    if sensor_logged:
        axes[5].plot(time, column(rows, 'sensor_signal_strength'), color='#7554a3',
                     label='EIT signal strength')
        axes[5].plot(time, column(rows, 'sensor_contact_threshold'), '--',
                     color='#a35050', label='Contact threshold')
        axes[5].set_ylabel('EIT signal / cutoff')
        flags = axes[5].twinx()
        flags.step(time, [row['sensor_contact'] == 'True' for row in rows],
                   where='post', color='#047857', label='EIT contact')
        flags.step(time, [row['sensor_fresh'] == 'True' for row in rows],
                   where='post', color='#888888', alpha=.7, linestyle=':', label='Fresh')
        flags.set_ylim(-.1, 1.2)
        flags.set_yticks([0, 1])
        flags.set_ylabel('Contact / fresh')
        axes[5].legend(loc='upper left', fontsize=8)
        flags.legend(loc='upper right', fontsize=8)
    if pattern_logged:
        axes[6].plot(time, column(rows, 'sensor_pattern_distance'), color='#7355a1',
                     label='Pattern distance')
        axes[6].plot(time, column(rows, 'sensor_pattern_support_limit'), '--',
                     color='#a35050', label='Model support limit')
        axes[6].set_ylabel('Pattern distance')
        axes[6].legend(loc='best', fontsize=8)
    previous = None
    for row, at in zip(rows, time):
        mode = row['mode']
        if mode != previous:
            if mode in ('preload', 'interact', 'retract'):
                for axis in axes:
                    axis.axvline(at, color='#777777', ls=':', lw=.8)
                axes[0].annotate(mode, (at, 1), xycoords=('data', 'axes fraction'),
                                 xytext=(2, -3), textcoords='offset points',
                                 va='top', fontsize=8, rotation=90)
            previous = mode
    for axis in axes:
        axis.grid(alpha=.25)
    fault_row = next(((row, at) for row, at in zip(rows, time)
                      if row['fault'] != 'none'), None)
    if fault_row:
        for axis in axes:
            axis.axvline(fault_row[1], color='#b42318', linewidth=1.5)
        axes[0].annotate('FAULT: '+fault_row[0]['fault'],
                         (fault_row[1], 1), xycoords=('data', 'axes fraction'),
                         xytext=(4, -24), textcoords='offset points',
                         color='#b42318', fontsize=9, rotation=90, va='top')
    release_row = next(((row, at) for row, at in zip(rows, time)
                        if row.get('trial_contact_released') == 'True'), None)
    if release_row:
        for axis in axes:
            axis.axvline(release_row[1], color='#b42318', linewidth=1.5)
        axes[0].annotate('JOYSTICK SLIP' if release_row[0].get('joystick_slipped') == 'True' else 'CONTACT RELEASE',
                         (release_row[1], 1),
                         xycoords=('data', 'axes fraction'), xytext=(4, -24),
                         textcoords='offset points', color='#b42318', fontsize=9,
                         rotation=90, va='top')
    axes[-1].set_xlim(-2., end + 2.)
    axes[-1].set_xlabel('Time from trial trace start (s)')
    result = trace_status(rows)
    if rows[-1]['mode'] != 'done':
        for axis in axes:
            axis.axvline(time[-1], color='#b42318', ls='--', lw=1.)
    fig.suptitle(f'{trace.parent.name}: {LABELS[condition]} — {result}')
    detail = ('ESP32 angle uses a ±35° endpoint interpolation; near-stop values are approximate. '
              if calibration else 'ESP32 values are raw ADC counts, not calibrated angle. ')
    detail += ('\nRequested force is a command. Franka external force estimates load at the stiffness frame.'
               '\nIt does not isolate joystick contact force.')
    if fault_row and sensor_logged:
        row = fault_row[0]
        detail += (f'\nFault sample: contact={row["sensor_contact"]}, '
                   f'candidate={row.get("sensor_contact_candidate", "--")}, '
                   f'fresh={row["sensor_fresh"]}, signal={float(row["sensor_signal_strength"]):.6f}, '
                   f'threshold={float(row["sensor_contact_threshold"]):.6f}')
        if pattern_logged:
            detail += (f', pattern={float(row["sensor_pattern_distance"]):.3f}/'
                       f'{float(row["sensor_pattern_support_limit"]):.3f}, '
                       f'radius={float(row["sensor_predicted_radius_mm"]):.1f} mm')
        detail += '.'
    fig.text(.5, .015, detail,
             ha='center', fontsize=8 if sensor_logged else 9)
    fig.subplots_adjust(left=.11, right=.90 if sensor_logged else .97,
                        bottom=.14 if sensor_logged else .12, top=.91, hspace=.12)
    output = trace.with_name(trace.stem + '_analysis.png')
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def comparison_event(rows, origin, engaged=False):
    event = next((row for row in rows if int(row['host_time_ns']) >= origin and (row.get('joystick_slipped') == 'True' or
                  row.get('trial_contact_released') == 'True' or
                  row.get('fault', 'none') != 'none')), None)
    if event is None:
        return None
    reason = ('joystick slip' if event.get('joystick_slipped') == 'True' else
              'contact release' if event.get('trial_contact_released') == 'True' else
              event.get('fault', 'fault'))
    time_s = (float(event['trial_time_s']) if engaged and event.get('trial_time_s')
              else (int(event['host_time_ns'])-origin)/1e9)
    return dict(time_s=time_s, reason=reason)


def mark_comparison_event(axes, metadata, condition, index=0):
    event = metadata.get('event')
    if event is None:
        return
    t = event['time_s']
    for axis in axes:
        axis.axvline(t, color=COLORS[condition], ls=':', lw=1.)
    axes[0].text(.01, .96-.13*index,
                 f'{LABELS[condition]}: {event["reason"]} at {t:.2f} s',
                 transform=axes[0].transAxes, color=COLORS[condition],
                 fontsize=8, va='top')


def comparison_limits(data):
    ends = [max(value[0][-1], (value[8].get('event') or {}).get('time_s', 0.))
            for value in data.values()]
    return -2., max(1., max(ends))+.5


def contact_state(rows):
    """Logged classifier decision; unknown/stale readings are gaps, not no-contact."""
    if not rows or 'sensor_contact' not in rows[0]:
        return None
    return np.array([float(row['sensor_contact']=='True')
                     if row.get('sensor_fresh','True')=='True' and
                        row.get('sensor_measurement_valid','True')=='True'
                     else np.nan for row in rows])


def draw_contact_lanes(axis, series):
    # Separate condition lanes keep identical binary traces visible.
    ticks=[];labels=[];available=False
    for index,(condition,time,values) in enumerate(series):
        offset=2.*(len(series)-1-index)
        ticks.append(offset+.5);labels.append(LABELS[condition])
        for level in [0.,1.]:
            axis.axhline(offset+level,color='#dddddd',lw=.6,zorder=0)
            axis.text(.995,offset+level,str(int(level)),transform=axis.get_yaxis_transform(),
                      ha='right',va='bottom',fontsize=8,color='#666666')
        if values is None:
            axis.text(.5,offset+.5,'Contact state was not logged',
                      transform=axis.get_yaxis_transform(),ha='center',fontsize=8)
        else:
            available=True
            axis.step(time,offset+values,where='post',color=COLORS[condition],lw=1.2)
    axis.set_yticks(ticks,labels);axis.set_ylim(-.25,2.*max(0,len(series)-1)+1.3)
    axis.set_ylabel('EIT contact\n0 = No; 1 = Yes')
    axis.grid(axis='x',alpha=.25)
    return available


def pair_data(traces, esp, calibration=None):
    data = {}
    for condition, trace in traces.items():
        rows = read_rows(trace)
        if not rows:
            continue
        active = [row for row in rows if row['mode'] in ('preload', 'interact')]
        engaged = bool(active)
        if not active:
            active = [row for row in rows if row['mode'] == 'approach']
        if not active:
            continue
        origin = int(active[0]['host_time_ns'])
        # trial_time starts at confirmed contact; account for the first logged
        # sample arriving later. Older traces without it use their first sample.
        if engaged and active[0].get('trial_time_s'):
            origin -= round(float(active[0]['trial_time_s'])*1e9)
        metadata = dict(status=trace_status(rows), origin='contact' if engaged else 'approach start',
                        event=comparison_event(rows, origin, engaged),
                        contact_state=contact_state(active))
        time = (column(active, 'trial_time_s') if engaged and all(row.get('trial_time_s') for row in active)
                else (np.array([int(row['host_time_ns']) for row in active]) - origin) / 1e9)
        tilt_axis = selected_axis(active)
        point = column(active, f'point_m_{tilt_axis}')
        reference = column(active, f'reference_m_{tilt_axis}')
        end = max(time[-1], (metadata['event'] or {}).get('time_s', 0.))
        sensor_t, x, y = sensor_window(esp, origin, -2., end)
        center_x, center_y = sensor_center(esp, origin)
        values = (measured_angles(x, y, calibration)[1 if tilt_axis == 'y' else 0]
                  if calibration else (x - center_x if tilt_axis == 'y' else y - center_y))
        # Missing ESP32 data must not suppress the saved controller failure.
        y_t, y_change = smooth_sensor(sensor_t, values)
        data[condition] = (time, point, reference, command_column(active),
                           y_t, y_change, palm_orientation(active), tilt_axis, metadata)
    return data


def draw_pair(repeat_folder, traces, esp, calibration=None):
    data = pair_data(traces, esp, calibration)
    if not data:
        return None
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for condition in data:
        time, point, reference, angle, y_t, y_change, _, tilt_axis, metadata = data[condition]
        color = COLORS[condition]
        label = LABELS[condition] + ' (' + trace_status(read_rows(traces[condition])) + ')'
        axes[0].plot(time, np.rad2deg(angle), '--', color=color, label=f'Desired {label}')
        axes[1].plot(time, (reference - point[0]) * 1000, '--', color=color,
                     label=f'Reference {label}')
        axes[1].plot(time, (point - point[0]) * 1000, color=color,
                     label=f'Measured {label}')
        if calibration:
            axes[0].plot(y_t, y_change, color=color, label=f'Actual {label}')
            inside = (y_t >= time[0]) & (y_t <= time[-1])
            axes[2].plot(y_t[inside], y_change[inside] -
                         np.interp(y_t[inside], time, np.rad2deg(angle)),
                         color=color, label=label)
        else:
            axes[2].plot(y_t, y_change, color=color, label=label)
        mark_comparison_event(axes, metadata, condition, list(data).index(condition))
    axes[0].set_ylabel('Joystick angle (deg)' if calibration else 'Virtual angle (deg)')
    axes[1].set_ylabel(f'{tilt_axis.upper()} displacement (mm)')
    axes[2].set_ylabel('Actual - desired (deg)' if calibration else
                       f'ESP32 {"X" if tilt_axis == "y" else "Y"} change (ADC counts)')
    axes[2].set_xlabel('Time from contact / approach start for unengaged trials (s)')
    axes[0].legend(loc='best')
    axes[1].legend(loc='best', fontsize=8, ncol=2)
    axes[2].legend(loc='best')
    for axis in axes:
        axis.axvline(0, color='#555555', ls=':', lw=.8)
        axis.grid(alpha=.25)
    axes[-1].set_xlim(*comparison_limits(data))
    fig.suptitle(f'{repeat_folder.name}: joystick condition comparison')
    fig.text(.5, .015, 'Endpoint angle interpolation is approximate near the stops.' if calibration else
             'ESP32 change is relative to its pre-interaction median; raw ADC counts are not calibrated to angle.',
             ha='center', fontsize=9)
    fig.subplots_adjust(left=.11, right=.97, bottom=.11, top=.91, hspace=.11)
    output = repeat_folder / 'comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def draw_orientation_pair(repeat_folder, data):
    available = [condition for condition in data if data[condition][6] is not None]
    if not available:
        return None
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for condition in available:
        time, _, _, _, _, _, orientation, tilt_axis, metadata = data[condition]
        desired, actual, shaft, perpendicular_error, target_error = orientation
        color = COLORS[condition]
        label = LABELS[condition] + ' (' + metadata['status'] + ')'
        mark_comparison_event(axes, metadata, condition, available.index(condition))
        axes[0].plot(time, desired, '--', color=color, label=f'Desired palm {label}')
        axes[0].plot(time, actual, color=color, label=f'Actual palm {label}')
        axes[0].plot(time, shaft, ':', color=color, alpha=.75,
                     label=f'Joystick shaft {label}')
        axes[1].plot(time, perpendicular_error, color=color,
                     label=f'Palm vs shaft {label}')
        axes[1].plot(time, target_error, '--', color=color,
                     label=f'Palm vs desired {label}')
    axes[0].set_ylabel(f'Base {tilt_axis.upper()} tilt (deg)')
    axes[1].set_ylabel('3D normal error (deg)')
    axes[1].set_xlabel('Time from contact / approach start for unengaged trials (s)')
    for axis in axes:
        axis.axvline(0, color='#555555', ls=':', lw=.8)
        axis.grid(alpha=.25)
        axis.legend(loc='best', fontsize=8, ncol=2)
    axes[-1].set_xlim(*comparison_limits(data))
    fig.suptitle(f'{repeat_folder.name}: palm orientation versus joystick shaft')
    fig.text(.5, .015, 'Zero palm–shaft error means the pad face is perpendicular to the measured shaft. '
             'Stale shaft samples are omitted; ADC-rail angles are bounded estimates.', ha='center', fontsize=8)
    fig.subplots_adjust(left=.11, right=.97, bottom=.12, top=.91, hspace=.12)
    output = repeat_folder / 'orientation_comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def draw_block(folder, pairs, calibration=None):
    if not pairs:
        return None
    has_orientation = any(any(data[condition][6] is not None for condition in data)
                          for _, data in pairs)
    rows_per_repeat = 3 if has_orientation else 2
    heights = ([2.,1.5,1.2] if has_orientation else [2.,1.2]) * len(pairs)
    fig, axes = plt.subplots(rows_per_repeat * len(pairs), 1,
                             figsize=(11, max(5., (6.5 if has_orientation else 4.8) * len(pairs))),
                             gridspec_kw={'height_ratios':heights},
                             sharex=True, squeeze=False)
    for index, (name, data) in enumerate(pairs):
        axis = axes[rows_per_repeat * index, 0]
        orientation_axis = axes[rows_per_repeat * index + 1, 0] if has_orientation else None
        contact_axis = axes[rows_per_repeat * index + rows_per_repeat - 1, 0]
        contact_series = [(condition,data[condition][0],data[condition][8].get('contact_state'))
                          for condition in CONDITIONS if condition in data]
        draw_contact_lanes(contact_axis,contact_series)
        for condition in CONDITIONS:
            if condition in data:
                y_t, y_change = data[condition][4:6]
                time, angle = data[condition][0], data[condition][3]
                status = data[condition][8]['status']
                label = LABELS[condition] + ' (' + status + ')'
                event_axes = [axis,contact_axis] + ([orientation_axis] if orientation_axis is not None else [])
                mark_comparison_event(event_axes, data[condition][8], condition, list(data).index(condition))
                if calibration:
                    axis.plot(time, np.rad2deg(angle), '--', color=COLORS[condition],
                              label=f'Desired {LABELS[condition]}')
                axis.plot(y_t, y_change, color=COLORS[condition],
                          label=f'Actual {label}' if calibration else label)
                if orientation_axis is not None and data[condition][6] is not None:
                    orientation_axis.plot(time, data[condition][6][3], color=COLORS[condition],
                                          label=LABELS[condition])
        tilt_axis = next(iter(data.values()))[7]
        axis.set_ylabel(f'{name}\n'+(f'Base {tilt_axis.upper()} angle (deg)' if calibration else 'ADC change'))
        axis.grid(alpha=.25)
        axis.legend(loc='best', fontsize=8)
        if orientation_axis is not None:
            orientation_axis.set_ylabel('Away from\nperpendicular (deg)')
            orientation_axis.axhline(0, color='#777777', lw=.8)
            orientation_axis.grid(alpha=.25)
            if orientation_axis.lines[1:]:
                orientation_axis.legend(loc='best', fontsize=8)
            else:
                orientation_axis.text(.5, .5, 'Palm orientation was not logged',
                                      transform=orientation_axis.transAxes, ha='center')
    axes[-1, 0].set_xlim(-2., max(comparison_limits(data)[1] for _, data in pairs))
    axes[-1, 0].set_xlabel('Time from contact / approach start for unengaged trials (s)')
    fig.suptitle('Joystick condition comparison: '+
                 ('angle, palm perpendicularity and EIT contact' if has_orientation else
                  ('Selected-axis angle' if calibration else 'ESP32 selected axis')+' and EIT contact by repeat'))
    fig.text(.5, .01, ('Zero perpendicularity error means the pad face is normal to the measured shaft; '
                      'ADC-rail angles are bounded estimates. Contact gaps = stale/invalid samples.' if has_orientation else
                      'Angle from ±35° endpoint interpolation; success requires independent review.') if calibration else
             'Raw ADC counts relative to pre-interaction median; no angle calibration or success claim.',
             ha='center', fontsize=9)
    fig.subplots_adjust(left=.12, right=.97, bottom=max(.08, .18 / len(pairs)), top=.93, hspace=.30)
    output = folder / 'block_comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def draw_run_summary(folder):
    """Always produce a diagnostic, including ready-only and recorder-start failures."""
    repeats = sorted(folder.glob('repeat_*')) or [folder]
    conditions = conditions_in_folder(folder)
    fig, axes = plt.subplots(2*len(repeats), len(conditions), figsize=(6*len(conditions), max(5, 4.7*len(repeats))),
                             gridspec_kw={'height_ratios':[2.,1.]*len(repeats)}, squeeze=False)
    for index, repeat in enumerate(repeats):
        for column_index, condition in enumerate(conditions):
            axis = axes[2*index, column_index]
            contact_axis = axes[2*index+1,column_index]
            candidates = sorted(repeat.glob(f'joystick_{condition}_*.csv'))
            rows = read_rows(candidates[-1]) if candidates else []
            axis.set_title(f'{repeat.name}: {LABELS[condition]}\n{trace_status(rows)}', fontsize=10)
            if not rows:
                ready = sorted(repeat.glob('joystick_ready_*.csv'))
                detail = 'No trial samples saved'
                if ready:
                    ready_rows = read_rows(ready[-1])
                    detail += '\nLast ready trace: ' + (ready_rows[-1].get('mode', 'unknown') if ready_rows else 'no samples')
                axis.text(.5, .5, detail, transform=axis.transAxes, ha='center')
                axis.set_axis_off()
                contact_axis.set_axis_off()
                continue
            tilt_axis = selected_axis(rows)
            times = (column(rows, 'host_time_ns') - int(rows[0]['host_time_ns'])) / 1e9
            command = np.where([row['mode'] in ('preload', 'interact') for row in rows],
                               command_column(rows), np.nan)
            axis.plot(times, np.rad2deg(command), '--', label='Desired (manipulation)')
            key = 'joystick_shaft_angle_y_rad' if tilt_axis == 'y' else 'joystick_shaft_angle_rad'
            if key in rows[0]:
                angles = column(rows, key)
                if 'joystick_shaft_fresh' in rows[0]:
                    angles = np.where([row['joystick_shaft_fresh']=='True' for row in rows], angles, np.nan)
                axis.plot(times, np.rad2deg(angles), label='Measured')
            axis.axvline(times[-1], color='#b42318' if rows[-1]['mode']!='done' else '#777777', ls=':')
            axis.set_xlabel('Time from trial trace start (s)')
            axis.set_ylabel(f'Base {tilt_axis.upper()} angle (deg)')
            axis.legend(fontsize=8);axis.grid(alpha=.25)
            draw_contact_lanes(contact_axis,[(condition,times,contact_state(rows))])
            contact_axis.set_xlim(axis.get_xlim())
            contact_axis.set_xlabel('Time from trial trace start (s)')
    result_path = folder/'block_result.json'
    result = json.loads(result_path.read_text()) if result_path.is_file() else {}
    detail = result.get('status', 'Saved traces; completion assessed separately for each condition')
    if result.get('error'):detail += ': ' + result['error']
    fig.suptitle(folder.name, fontsize=11)
    fig.text(.5, .025, textwrap.fill(detail, 130), ha='center', fontsize=8)
    fig.tight_layout(rect=(0, .12, 1, .92))
    output = folder/'run_summary.png'
    fig.savefig(output, dpi=150);plt.close(fig)
    return output


def plot_block(folder):
    # Save the summary first so one malformed trace cannot hide all diagnostics.
    outputs = [draw_run_summary(folder)]
    esp = read_esp(folder / 'joystick_esp32.csv')
    calibration = load_calibration(folder)
    pairs = []
    for repeat_folder in sorted(folder.glob('repeat_*')):
        traces = {}
        for condition in CONDITIONS:
            candidates = sorted(repeat_folder.glob(f'joystick_{condition}_*.csv'))
            if candidates:
                traces[condition] = candidates[-1]
                try:
                    output = draw_trial(candidates[-1], esp, calibration)
                    if output:outputs.append(output)
                except (OSError, ValueError, KeyError, IndexError) as exc:
                    print(f'Could not plot {candidates[-1].name}: {exc}', file=sys.stderr)
        if traces:
            try:
                data = pair_data(traces, esp, calibration)
                output = draw_pair(repeat_folder, traces, esp, calibration)
                if output:
                    outputs.append(output)
                    pairs.append((repeat_folder.name, data))
                output = draw_orientation_pair(repeat_folder, data)
                if output:outputs.append(output)
            except (OSError, ValueError, KeyError, IndexError) as exc:
                print(f'Could not compare {repeat_folder.name}: {exc}', file=sys.stderr)
    output = draw_block(folder, pairs, calibration)
    if output:
        outputs.append(output)
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path, help='Joystick block folder containing joystick_esp32.csv')
    args = parser.parse_args()
    for output in plot_block(args.folder):
        print(output)


if __name__ == '__main__':
    main()
