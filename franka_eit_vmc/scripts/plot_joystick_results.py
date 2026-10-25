#!/usr/bin/env python3
"""Plot controller traces beside the independent ESP32 recording in a block folder."""
import argparse
import csv
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


def column(rows, key):
    return np.array([float(row[key]) for row in rows])


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


def draw_trial(trace, esp):
    rows = read_rows(trace)
    if not rows:
        return None
    origin = trial_origin(rows)
    time = (np.array([int(row['host_time_ns']) for row in rows]) - origin) / 1e9
    end = max(1., time[-1])
    condition = 'eit' if '_eit_' in trace.name else 'baseline'
    color = COLORS[condition]
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(time, np.rad2deg(column(rows, 'theta_d_rad')), color=color)
    axes[0].set_ylabel('Virtual angle (deg)')
    point = column(rows, 'point_m_x')
    reference = column(rows, 'reference_m_x')
    axes[1].plot(time, (reference - point[0]) * 1000, '--', color=color, label='Reference')
    axes[1].plot(time, (point - point[0]) * 1000, color=color, label='Measured point')
    axes[1].set_ylabel('X displacement (mm)')
    axes[1].legend(loc='upper right')
    axes[2].plot(time, column(rows, 'requested_force_N_x'), color=color, label='X')
    axes[2].plot(time, column(rows, 'requested_force_N_z'), color='#555555', label='Z')
    axes[2].set_ylabel('Requested force (N)')
    axes[2].legend(loc='upper right')
    sensor_t, x, y = sensor_window(esp, origin, -2., end + 2.)
    center_x, center_y = sensor_center(esp, origin)
    for values, center, label, style in ((x, center_x, 'ESP32 X', '--'),
                                         (y, center_y, 'ESP32 Y', '-')):
        if np.isfinite(center):
            plot_t, plot_y = smooth_sensor(sensor_t, values - center)
            axes[3].plot(plot_t, plot_y, style, label=label)
    axes[3].set_ylabel('ADC change (counts)')
    axes[3].legend(loc='best')
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
    axes[-1].set_xlim(-2., end + 2.)
    axes[-1].set_xlabel('Time from trial trace start (s)')
    fault = next((row['fault'] for row in rows if row['fault'] != 'none'), None)
    result = f'fault: {fault}' if fault else ('retract incomplete' if rows[-1]['retract_incomplete'] == 'True'
                                              else rows[-1]['mode'])
    fig.suptitle(f'{trace.parent.name}: {LABELS[condition]} — {result}')
    fig.text(.5, .015, 'ESP32 values are raw ADC counts, not calibrated angle. Requested force is not measured contact force.',
             ha='center', fontsize=9)
    fig.subplots_adjust(left=.11, right=.97, bottom=.10, top=.91, hspace=.10)
    output = trace.with_name(trace.stem + '_analysis.png')
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def pair_data(traces, esp):
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
        sensor_t, _, y = sensor_window(esp, origin, -2., 15.)
        _, center_y = sensor_center(esp, origin)
        if not np.isfinite(center_y):
            continue
        y_t, y_change = smooth_sensor(sensor_t, y - center_y)
        data[condition] = (time, point, reference, column(active, 'theta_d_rad'),
                           y_t, y_change)
    return data


def draw_pair(repeat_folder, traces, esp):
    data = pair_data(traces, esp)
    if len(data) != 2:
        return None
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for condition in ('baseline', 'eit'):
        time, point, reference, angle, y_t, y_change = data[condition]
        color = COLORS[condition]
        label = LABELS[condition]
        axes[0].plot(time, np.rad2deg(angle), color=color, label=label)
        axes[1].plot(time, (reference - point[0]) * 1000, '--', color=color,
                     label=f'Reference {label}')
        axes[1].plot(time, (point - point[0]) * 1000, color=color,
                     label=f'Measured {label}')
        axes[2].plot(y_t, y_change, color=color, label=label)
    axes[0].set_ylabel('Virtual angle (deg)')
    axes[1].set_ylabel('X displacement (mm)')
    axes[2].set_ylabel('ESP32 Y change (ADC counts)')
    axes[2].set_xlabel('Time from interaction start (s)')
    axes[0].legend(loc='best')
    axes[1].legend(loc='best', fontsize=8, ncol=2)
    axes[2].legend(loc='best')
    for axis in axes:
        axis.axvline(0, color='#555555', ls=':', lw=.8)
        axis.grid(alpha=.25)
    axes[-1].set_xlim(-2, 15)
    fig.suptitle(f'{repeat_folder.name}: joystick baseline versus EIT')
    fig.text(.5, .015, 'ESP32 change is relative to its pre-interaction median; raw ADC counts are not calibrated to angle.',
             ha='center', fontsize=9)
    fig.subplots_adjust(left=.11, right=.97, bottom=.11, top=.91, hspace=.11)
    output = repeat_folder / 'comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def draw_block(folder, pairs):
    if not pairs:
        return None
    fig, axes = plt.subplots(len(pairs), 1, figsize=(11, max(3.5, 2.6 * len(pairs))),
                             sharex=True, squeeze=False)
    for axis, (name, data) in zip(axes[:, 0], pairs):
        for condition in ('baseline', 'eit'):
            if condition in data:
                y_t, y_change = data[condition][4:]
                axis.plot(y_t, y_change, color=COLORS[condition],
                          label=LABELS[condition])
        axis.set_ylabel(f'{name}\nADC change')
        axis.grid(alpha=.25)
        axis.legend(loc='best', fontsize=8)
    axes[-1, 0].set_xlim(-2, 15)
    axes[-1, 0].set_xlabel('Time from interaction start (s)')
    fig.suptitle('Joystick baseline versus EIT: ESP32 Y by repeat')
    fig.text(.5, .01, 'Raw ADC counts relative to pre-interaction median; no angle calibration or success claim.',
             ha='center', fontsize=9)
    fig.subplots_adjust(left=.12, right=.97, bottom=max(.08, .32 / len(pairs)), top=.91, hspace=.22)
    output = folder / 'block_comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def plot_block(folder):
    esp = read_esp(folder / 'joystick_esp32.csv')
    outputs = []
    pairs = []
    for repeat_folder in sorted(folder.glob('repeat_*')):
        traces = {}
        for condition in ('baseline', 'eit'):
            candidates = sorted(repeat_folder.glob(f'joystick_{condition}_*.csv'))
            if candidates:
                traces[condition] = candidates[-1]
                output = draw_trial(candidates[-1], esp)
                if output:
                    outputs.append(output)
        if len(traces) == 2:
            output = draw_pair(repeat_folder, traces, esp)
            if output:
                outputs.append(output)
                pairs.append((repeat_folder.name, pair_data(traces, esp)))
    output = draw_block(folder, pairs)
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
