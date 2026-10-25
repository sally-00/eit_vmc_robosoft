#!/usr/bin/env python3
"""Plot controller traces beside the independent ESP32 recording in a block folder."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


COLORS = {'baseline': '#1764ab', 'eit': '#d35400'}
LABELS = {'baseline': 'without EIT', 'eit': 'with EIT'}


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def read_esp(path):
    rows = read_rows(path)
    if not rows:
        raise ValueError(f'No ESP32 samples in {path}')
    return {key: np.array([int(row[key]) for row in rows], dtype=np.int64)
            for key in ('host_unix_ns', 'x_raw', 'y_raw')}


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
    actual_tilt = np.rad2deg(np.arctan2(-actual[:, 0], -actual[:, 2]))
    desired_tilt = np.rad2deg(np.arctan2(-desired[:, 0], -desired[:, 2]))
    shaft_tilt = np.rad2deg(shaft)
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
    origin = trial_origin(rows)
    time = (np.array([int(row['host_time_ns']) for row in rows]) - origin) / 1e9
    end = max(1., time[-1])
    condition = 'eit' if '_eit_' in trace.name else 'baseline'
    color = COLORS[condition]
    sensor_logged = 'sensor_contact' in rows[0]
    pattern_logged = 'sensor_pattern_distance' in rows[0]
    panels = 7 if pattern_logged else 6 if sensor_logged else 5
    fig, axes = plt.subplots(panels, 1, figsize=(11, 15 if pattern_logged else 13 if sensor_logged else 11),
                            sharex=True)
    axes[0].plot(time, np.rad2deg(column(rows, 'theta_d_rad')), color=color)
    axes[0].set_ylabel('Virtual angle (deg)')
    point = column(rows, 'point_m_x')
    reference = column(rows, 'reference_m_x')
    axes[1].plot(time, (reference - point[0]) * 1000, '--', color=color, label='Reference')
    axes[1].plot(time, (point - point[0]) * 1000, color=color, label='Measured point')
    axes[1].set_ylabel('X displacement (mm)')
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
            if label == 'Base X angle':
                axes[0].plot(plot_t, plot_y, color='#15936e', alpha=.8,
                             label='Measured Base X angle')
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
    axes[-1].set_xlim(-2., end + 2.)
    axes[-1].set_xlabel('Time from trial trace start (s)')
    fault = next((row['fault'] for row in rows if row['fault'] != 'none'), None)
    result = f'fault: {fault}' if fault else ('retract incomplete' if rows[-1]['retract_incomplete'] == 'True'
                                              else rows[-1]['mode'])
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


def pair_data(traces, esp, calibration=None):
    data = {}
    for condition, trace in traces.items():
        rows = read_rows(trace)
        origin = interact_origin(rows)
        if origin is None:
            continue
        active = [row for row in rows if row['mode'] == 'interact']
        time = (np.array([int(row['host_time_ns']) for row in active]) - origin) / 1e9
        point = column(active, 'point_m_x')
        reference = column(active, 'reference_m_x')
        sensor_t, x, y = sensor_window(esp, origin, -2., 15.)
        center_x, center_y = sensor_center(esp, origin)
        if not np.isfinite(center_y) or not np.isfinite(center_x):
            continue
        values = (measured_angles(x, y, calibration)[0]
                  if calibration else y - center_y)
        y_t, y_change = smooth_sensor(sensor_t, values)
        data[condition] = (time, point, reference, column(active, 'theta_d_rad'),
                           y_t, y_change, palm_orientation(active))
    return data


def draw_pair(repeat_folder, traces, esp, calibration=None):
    data = pair_data(traces, esp, calibration)
    if len(data) != 2:
        return None
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for condition in ('baseline', 'eit'):
        time, point, reference, angle, y_t, y_change, _ = data[condition]
        color = COLORS[condition]
        label = LABELS[condition]
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
    axes[0].set_ylabel('Joystick angle (deg)' if calibration else 'Virtual angle (deg)')
    axes[1].set_ylabel('X displacement (mm)')
    axes[2].set_ylabel('Actual - desired (deg)' if calibration else
                       'ESP32 Y change (ADC counts)')
    axes[2].set_xlabel('Time from interaction start (s)')
    axes[0].legend(loc='best')
    axes[1].legend(loc='best', fontsize=8, ncol=2)
    axes[2].legend(loc='best')
    for axis in axes:
        axis.axvline(0, color='#555555', ls=':', lw=.8)
        axis.grid(alpha=.25)
    axes[-1].set_xlim(-2, 15)
    fig.suptitle(f'{repeat_folder.name}: joystick baseline versus EIT')
    fig.text(.5, .015, 'Endpoint angle interpolation is approximate near the stops.' if calibration else
             'ESP32 change is relative to its pre-interaction median; raw ADC counts are not calibrated to angle.',
             ha='center', fontsize=9)
    fig.subplots_adjust(left=.11, right=.97, bottom=.11, top=.91, hspace=.11)
    output = repeat_folder / 'comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def draw_orientation_pair(repeat_folder, data):
    if any(condition not in data or data[condition][6] is None
           for condition in ('baseline', 'eit')):
        return None
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for condition in ('baseline', 'eit'):
        time, _, _, _, _, _, orientation = data[condition]
        desired, actual, shaft, perpendicular_error, target_error = orientation
        color = COLORS[condition]
        label = LABELS[condition]
        axes[0].plot(time, desired, '--', color=color, label=f'Desired palm {label}')
        axes[0].plot(time, actual, color=color, label=f'Actual palm {label}')
        axes[0].plot(time, shaft, ':', color=color, alpha=.75,
                     label=f'Joystick shaft {label}')
        axes[1].plot(time, perpendicular_error, color=color,
                     label=f'Palm vs shaft {label}')
        axes[1].plot(time, target_error, '--', color=color,
                     label=f'Palm vs desired {label}')
    axes[0].set_ylabel('Base X tilt (deg)')
    axes[1].set_ylabel('3D normal error (deg)')
    axes[1].set_xlabel('Time from interaction start (s)')
    for axis in axes:
        axis.axvline(0, color='#555555', ls=':', lw=.8)
        axis.grid(alpha=.25)
        axis.legend(loc='best', fontsize=8, ncol=2)
    axes[-1].set_xlim(0, 15)
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
    rows_per_repeat = 2 if has_orientation else 1
    fig, axes = plt.subplots(rows_per_repeat * len(pairs), 1,
                             figsize=(11, max(3.5, (4.5 if has_orientation else 2.6) * len(pairs))),
                             sharex=True, squeeze=False)
    for index, (name, data) in enumerate(pairs):
        axis = axes[rows_per_repeat * index, 0]
        orientation_axis = axes[rows_per_repeat * index + 1, 0] if has_orientation else None
        for condition in ('baseline', 'eit'):
            if condition in data:
                y_t, y_change = data[condition][4:6]
                time, angle = data[condition][0], data[condition][3]
                if calibration:
                    axis.plot(time, np.rad2deg(angle), '--', color=COLORS[condition],
                              label=f'Desired {LABELS[condition]}')
                axis.plot(y_t, y_change, color=COLORS[condition],
                          label=f'Actual {LABELS[condition]}' if calibration else LABELS[condition])
                if orientation_axis is not None and data[condition][6] is not None:
                    orientation_axis.plot(time, data[condition][6][3], color=COLORS[condition],
                                          label=LABELS[condition])
        axis.set_ylabel(f'{name}\n'+('Base X angle (deg)' if calibration else 'ADC change'))
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
    axes[-1, 0].set_xlim(-2, 15)
    axes[-1, 0].set_xlabel('Time from interaction start (s)')
    fig.suptitle('Joystick baseline versus EIT: '+
                 ('angle and palm perpendicularity' if has_orientation else
                  ('Base X angle' if calibration else 'ESP32 Y')+' by repeat'))
    fig.text(.5, .01, ('Zero perpendicularity error means the pad face is normal to the measured shaft; '
                      'ADC-rail angles are bounded estimates.' if has_orientation else
                      'Angle from ±35° endpoint interpolation; success requires independent review.') if calibration else
             'Raw ADC counts relative to pre-interaction median; no angle calibration or success claim.',
             ha='center', fontsize=9)
    fig.subplots_adjust(left=.12, right=.97, bottom=max(.08, .18 / len(pairs)), top=.91, hspace=.22)
    output = folder / 'block_comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def plot_block(folder):
    esp = read_esp(folder / 'joystick_esp32.csv')
    calibration = load_calibration(folder)
    outputs = []
    pairs = []
    for repeat_folder in sorted(folder.glob('repeat_*')):
        traces = {}
        for condition in ('baseline', 'eit'):
            candidates = sorted(repeat_folder.glob(f'joystick_{condition}_*.csv'))
            if candidates:
                traces[condition] = candidates[-1]
                output = draw_trial(candidates[-1], esp, calibration)
                if output:
                    outputs.append(output)
        if len(traces) == 2:
            data = pair_data(traces, esp, calibration)
            output = draw_pair(repeat_folder, traces, esp, calibration)
            if output:
                outputs.append(output)
                pairs.append((repeat_folder.name, data))
            output = draw_orientation_pair(repeat_folder, data)
            if output:
                outputs.append(output)
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
