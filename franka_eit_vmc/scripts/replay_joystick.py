#!/usr/bin/env python3
"""Render offline, evidence-linked robot/pad/joystick MP4 replays. No ROS or robot I/O.

Requires numpy, scipy, matplotlib, PyYAML and OpenCV with an H.264 encoder.
Run: python3 replay_joystick.py ~/eit_trials/joystick/<block> --output <block>/vmc_replay
"""
import argparse
import csv
import datetime
import html
import json
import shutil
import textwrap
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation


XYZ = 'xyz'
CONTACT_MODES = ('preload', 'interact')
COLORS = dict(pad='#64748b', shaft='#16a34a', attachment='#db2777',
              reference='#f59e0b', alignment='#7c3aed', robot='#2563eb')


def between_vectors(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    cross = np.cross(a, b)
    sine, cosine = np.linalg.norm(cross), np.clip(np.dot(a, b), -1., 1.)
    if sine < 1e-10:
        if cosine > 0:
            return np.eye(3)
        axis = np.eye(3)[np.argmin(np.abs(a))]
        axis = np.cross(a, axis)
        return Rotation.from_rotvec(np.pi * axis / np.linalg.norm(axis)).as_matrix()
    return Rotation.from_rotvec(cross / sine * np.arctan2(sine, cosine)).as_matrix()


def spring_curve(start, end, width=.002, turns=7):
    """Coil centreline with exact endpoints; coil width is a drawing convention."""
    start, end = np.asarray(start), np.asarray(end)
    delta = end - start
    length = np.linalg.norm(delta)
    if not np.isfinite(length):
        return np.empty((0, 3))
    if length < 1e-7:
        return np.array([start, end])
    direction = delta / length
    side = np.cross(direction, np.eye(3)[np.argmin(np.abs(direction))])
    side /= np.linalg.norm(side)
    other = np.cross(direction, side)
    u = np.linspace(0, 1, 100)
    envelope = np.minimum(np.minimum(u / .15, (1-u) / .15), 1.)
    phase = 2 * np.pi * turns * u
    return (start + u[:, None] * delta + min(width, length * .22) * envelope[:, None] *
            (np.cos(phase)[:, None] * side + np.sin(phase)[:, None] * other))


def rows_from(path):
    with path.open(newline='') as stream:
        return [r for r in csv.DictReader(stream)
                if None not in r and all(v is not None for v in r.values())]


def vectors(rows, prefix):
    return np.array([[float(r[f'{prefix}_{a}']) for a in XYZ] for r in rows])


def fk(q, kinematics):
    """Nominal FR3 chain through link 7; tool transform is fitted/validated below."""
    transform = np.eye(4)
    points = [np.zeros(3)]
    for i, angle in enumerate(q, 1):
        k = kinematics[f'joint{i}']['kinematic']
        origin = np.eye(4)
        origin[:3, 3] = [k[a] for a in XYZ]
        origin[:3, :3] = Rotation.from_euler('xyz', [k[a] for a in ('roll', 'pitch', 'yaw')]).as_matrix()
        turn = np.eye(4)
        turn[:3, :3] = Rotation.from_rotvec([0, 0, angle]).as_matrix()
        transform = transform @ origin @ turn
        points.append(transform[:3, 3].copy())
    return np.array(points), transform[:3, :3]


def fit_pad_rotation(trials):
    """Fit constant flange->pad orientation from normals and engaged attachments.

    Normal alone leaves yaw undetermined. EIT attachment vectors supply yaw;
    otherwise use the repository's nominal -90 degree pad clocking.
    """
    source, target = [], []
    for tr in trials:
        for i in range(0, len(tr.rows), 4):
            source.append([0., 0., 1.])
            target.append(tr.flange_r[i].T @ tr.normal[i])
            if tr.engaged[i] and np.linalg.norm(tr.attachment[i]) > .001:
                source.append(tr.attachment[i] / np.linalg.norm(tr.attachment[i]))
                d = tr.flange_r[i].T @ (tr.point[i] - tr.pad[i])
                target.append(d / np.linalg.norm(d))
    if not source:
        raise ValueError('No valid orientation observations')
    # If no nonzero attachment was recorded, yaw is schematic but pad normal is measured.
    if np.linalg.matrix_rank(source) < 2:
        n = np.mean(target, axis=0)
        n /= np.linalg.norm(n)
        u = np.array([0., -1., 0.])
        u -= n * np.dot(u, n)
        u /= np.linalg.norm(u)
        return np.column_stack((u, np.cross(n, u), n)), 'nominal clocking; measured normal'
    rotation, _ = Rotation.align_vectors(np.array(target), np.array(source))
    return rotation.as_matrix(), 'fitted from logged normals and engaged attachment vectors'


class Trial:
    def __init__(self, path, kinematics):
        self.path = path
        self.block = path.parent.parent
        self.rows = rows_from(path)
        if len(self.rows) < 2:
            raise ValueError(f'Too few complete samples: {path}')
        self.config = json.loads((self.block / 'block.json').read_text())
        self.params = self.config.get('checked_controller_parameters', {})
        self.alignment_enabled = self.params.get('joystick_alignment_enabled', True)
        self.task_spring_uses_shaft = self.params.get('joystick_task_spring_uses_shaft', False)
        self.update_eit_attachment = self.params.get('joystick_update_eit_attachment', False)
        self.ns = np.array([int(r['host_time_ns']) for r in self.rows], dtype=np.int64)
        if np.any(np.diff(self.ns) <= 0):
            raise ValueError(f'Nonmonotonic host timestamps: {path}')
        self.time = (self.ns - self.ns[0]) / 1e9
        self.modes = np.array([r['mode'] for r in self.rows])
        self.engaged = np.isin(self.modes, CONTACT_MODES)
        self.axis = self.rows[-1].get('joystick_axis', self.config.get('axis', 'x'))
        self.pad = vectors(self.rows, 'pad_m')
        self.normal = vectors(self.rows, 'pad_normal_base')
        self.desired_normal = vectors(self.rows, 'desired_pad_normal_base')
        self.point = vectors(self.rows, 'point_m')
        self.reference = vectors(self.rows, 'reference_m')
        self.attachment = vectors(self.rows, 'attachment_m')
        self.force = vectors(self.rows, 'requested_force_N')
        self.q = np.array([[float(r[f'q{i}_rad']) for i in range(1, 8)] for r in self.rows])
        geometry = [fk(q, kinematics) for q in self.q]
        self.links = np.array([g[0] for g in geometry])
        self.flange_r = np.array([g[1] for g in geometry])
        self.pivot = np.array(self.params.get('joystick_pivot_m', [.45, 0., .035]))
        self.length = float(self.params.get('joystick_length', .02))
        self.depth = float(self.params.get('joystick_inward_arc_m', 0.))
        self.shaft = np.column_stack((self.col('joystick_shaft_angle_rad'),
                                      self.col('joystick_shaft_angle_y_rad')))
        self.shaft_valid = self.flag('joystick_shaft_fresh')
        self.command = np.column_stack((self.col('theta_d_rad'), self.col('theta_d_y_rad')))
        self.shaft_axis = np.column_stack((np.tan(self.shaft), np.ones(len(self.rows))))
        self.shaft_axis /= np.linalg.norm(self.shaft_axis, axis=1)[:, None]
        self.tip = self.pivot + self.length * self.shaft_axis
        self.tip[~self.shaft_valid] = np.nan
        self.normal_gap_mm = np.sum((self.tip - self.pad) * self.normal, axis=1) * 1000
        self.alignment_error = np.rad2deg(np.arccos(np.clip(
            -np.sum(self.normal * self.shaft_axis, axis=1), -1, 1)))
        self.alignment_error[~self.shaft_valid] = np.nan
        self.target_orientation_error = np.rad2deg(np.arccos(np.clip(
            np.sum(self.normal * self.desired_normal, axis=1), -1, 1)))
        self.condition = ('baseline' if '_baseline_' in path.name else
                          'EIT' if '_eit_' in path.name else
                          'kinematic' if '_kinematic_' in path.name else 'ready')
        self.label = self.block.name.split('_2026')[0] + ' / ' + self.condition
        self.raw_angles = self.load_raw_angles()

    def col(self, key, default=np.nan):
        return np.array([float(r.get(key, default)) for r in self.rows])

    def flag(self, key):
        return np.array([r.get(key) == 'True' for r in self.rows])

    def load_raw_angles(self):
        """Independent CSV uses preserved host receipt times, with no fitted time shift."""
        path = self.block / 'joystick_esp32.csv'
        calibration_path = self.block / 'joystick_adc_calibration.json'
        if not path.exists() or not calibration_path.exists():
            return None
        calibration = json.loads(calibration_path.read_text())
        mapping = calibration['mapping']
        records = rows_from(path)
        ns = np.array([int(r['host_unix_ns']) for r in records], dtype=np.int64)
        keep = (ns >= self.ns[0] - 100_000_000) & (ns <= self.ns[-1] + 100_000_000)
        records = [r for r, use in zip(records, keep) if use]
        ns = ns[keep]
        if not len(ns):
            return None
        angles = []
        for base, raw_name in (('base_x_deg', 'y_raw'), ('base_y_deg', 'x_raw')):
            spec = mapping[base]
            raw = np.array([float(r[raw_name]) for r in records])
            mid, plus, minus = [float(spec[k]) for k in ('center_raw', 'plus_limit_raw', 'minus_limit_raw')]
            limit = float(calibration['nominal_limit_deg'])
            angle = np.where(raw <= mid, limit * (mid - raw) / (mid - plus),
                             -limit * (raw - mid) / (minus - mid))
            angles.append(np.clip(angle, -limit, limit))
        return (ns - self.ns[0]) / 1e9, np.array(angles).T

    def finish_geometry(self, pad_rotation, rotation_source):
        self.rotation_source = rotation_source
        self.pad_rotation = self.flange_r @ pad_rotation
        self.mount_top = self.pad - .05 * self.normal
        # Fit one constant tool displacement, keeping the nominal robot base fixed.
        offset = np.einsum('nji,nj->ni', self.flange_r, self.pad - self.links[:, -1])
        self.tool_offset = np.mean(offset, axis=0)
        predicted = self.links[:, -1] + np.einsum('nij,j->ni', self.flange_r, self.tool_offset)
        self.fk_residual_mm = np.linalg.norm(predicted - self.pad, axis=1) * 1000
        self.material_attachment = self.pad + np.einsum('nij,nj->ni', self.pad_rotation, self.attachment)
        # Match measured normal exactly for the disk; FK supplies only in-plane clocking.
        u = self.pad_rotation[:, :, 0]
        u = u - np.sum(u * self.normal, axis=1)[:, None] * self.normal
        u /= np.linalg.norm(u, axis=1)[:, None]
        self.pad_rotation = np.stack((u, np.cross(self.normal, u), self.normal), axis=2)
        self.alignment_center = vectors(self.rows, 'alignment_centre_m')
        self.alignment_target = vectors(self.rows, 'alignment_target_m')
        self.alignment_half_span = float(self.params.get('joystick_alignment_half_span_m', .16))
        # Core captures hold_rotation at StartTrial. Full desired yaw is not
        # logged: infer it from the first approach pose, preserving logged normals.
        approach = np.flatnonzero(self.modes == 'approach')
        first = approach[0] if len(approach) else 0
        hold = between_vectors(self.normal[first], self.desired_normal[first]) @ self.pad_rotation[first]
        desired = []
        for i, shaft in enumerate(self.shaft_axis):
            q = between_vectors([0., 0., 1.], shaft) @ hold
            q = between_vectors(q[:, 2], self.desired_normal[i]) @ q
            desired.append(q)
        self.alignment_target_rotation = np.array(desired)
        self.alignment_local_arms = self.alignment_half_span * np.array(
            [[1.,0.,0.], [-1.,0.,0.], [0.,1.,0.], [0.,-1.,0.]])
        self.spring_actual_anchors = self.alignment_center[:, None, :] + np.einsum(
            'nij,kj->nki', self.pad_rotation, self.alignment_local_arms)
        self.spring_target_anchors = self.alignment_target[:, None, :] + np.einsum(
            'nij,kj->nki', self.alignment_target_rotation, self.alignment_local_arms)
        # In inward_arc, reference = task_pivot + (L-depth)*commanded_axis.
        # The task pivot is contact-defined in palm mode and the physical pivot
        # in shaft mode. Recover it from the recorded reference in either case.
        self.virtual_radius = self.length-self.depth
        self.spring_start = (self.pivot + self.virtual_radius*self.shaft_axis
                             if self.task_spring_uses_shaft else self.material_attachment)
        if self.task_spring_uses_shaft:
            # The coil depicts the XY shaft-error part of the task spring.
            # Its Z part holds the pad at the contact-defined height.
            self.spring_start[:, 2] = self.reference[:, 2]
        self.command_axis = np.column_stack((np.tan(self.command), np.ones(len(self.rows))))
        self.command_axis /= np.linalg.norm(self.command_axis, axis=1)[:, None]
        if self.engaged.any():
            candidates = self.reference[self.engaged] - self.virtual_radius*self.command_axis[self.engaged]
            self.virtual_pivot = np.median(candidates, axis=0)
            self.virtual_tip = self.virtual_pivot + self.virtual_radius*self.command_axis
            self.virtual_reference_residual_mm = np.linalg.norm(
                self.virtual_tip[self.engaged]-self.reference[self.engaged],axis=1)*1000
        else:
            self.virtual_pivot = np.full(3,np.nan)
            self.virtual_tip = np.full_like(self.reference,np.nan)
            self.virtual_reference_residual_mm = np.array([])

    def metrics(self):
        mask = (self.modes == 'interact') & self.shaft_valid
        selected = 1 if self.axis == 'y' else 0
        error = np.rad2deg(self.shaft[:, selected] - self.command[:, selected])
        faults = [(float(t), r['fault']) for t, r in zip(self.time, self.rows) if r['fault'] != 'none']
        mode_changes = []
        for i, row in enumerate(self.rows):
            if i == 0 or row['mode'] != self.rows[i - 1]['mode']:
                mode_changes.append(dict(time_s=float(self.time[i]), mode=row['mode']))
        def stat(a, mask=mask):
            a = np.asarray(a)[mask]
            a = a[np.isfinite(a)]
            return dict(median=float(np.median(a)), max=float(np.max(a)), min=float(np.min(a))) if len(a) else None
        dt = np.diff(self.time)
        angular_step = np.diff(self.q, axis=0)
        speed = np.abs(angular_step / dt[:, None])
        result = dict(source=str(self.path), block=self.block.name, condition=self.condition,
                      axis=self.axis, depth_mm=self.depth * 1000, duration_s=float(self.time[-1]),
                      samples=len(self.rows), final_mode=self.modes[-1],
                      alignment_enabled=self.alignment_enabled,
                      task_spring_uses_shaft=self.task_spring_uses_shaft,
                      update_eit_attachment=self.update_eit_attachment,
                      first_fault=dict(time_s=faults[0][0], name=faults[0][1]) if faults else None,
                      trial_contact_released=bool(self.flag('trial_contact_released')[-1]),
                      joystick_slipped=bool(self.flag('joystick_slipped')[-1]),
                      retract_incomplete=bool(self.flag('retract_incomplete')[-1]),
                      mode_changes=mode_changes, controller_dt_ms=stat(dt * 1000, np.ones(len(dt), bool)),
                      fk_position_residual_mm=stat(self.fk_residual_mm, np.ones(len(self.rows), bool)),
                      fitted_link7_to_pad_m=self.tool_offset.tolist(), orientation_source=self.rotation_source,
                      joint_speed_from_host_samples_max_rad_s=float(np.max(speed)),
                      interact_angle_rmse_deg=float(np.sqrt(np.mean(error[mask] ** 2))) if mask.any() else None,
                      measured_x_deg=stat(np.rad2deg(self.shaft[:, 0])),
                      measured_y_deg=stat(np.rad2deg(self.shaft[:, 1])),
                      alignment_error_deg=stat(self.alignment_error),
                      pad_vs_desired_normal_error_deg=stat(self.target_orientation_error),
                      modeled_tip_to_pad_plane_mm=stat(self.normal_gap_mm),
                      requested_force_norm_N=stat(np.linalg.norm(self.force, axis=1)),
                      external_force_norm_N=stat(np.linalg.norm(vectors(self.rows, 'external_force_N'), axis=1)),
                      attachment_radius_mm=stat(np.linalg.norm(self.attachment, axis=1) * 1000),
                      contact_false_fraction=float(np.mean(~self.flag('sensor_contact')[mask])) if mask.any() else None,
                      shaft_clipped_fraction=float(np.mean(self.flag('joystick_shaft_clipped')[mask])) if mask.any() else None,
                      shaft_stale_fraction=float(np.mean(~self.shaft_valid[self.modes == 'interact'])) if (self.modes == 'interact').any() else None,
                      slider_stop_fraction=float(np.mean(self.flag('joystick_slider_at_stop')[mask])) if mask.any() else None,
                      task_spring_scale=stat(self.col('joystick_task_spring_scale')))
        result['command_phase_angles_deg'] = {}
        for label, phase in (('positive_hold', self.command[:, selected] > np.deg2rad(18)),
                             ('negative_hold', self.command[:, selected] < np.deg2rad(-18))):
            result['command_phase_angles_deg'][label] = stat(np.rad2deg(self.shaft[:, selected]), mask & phase)
        if self.raw_angles and mask.any():
            t, angles = self.raw_angles
            sampled = np.column_stack([np.interp(self.time, t, angles[:, j]) for j in range(2)])
            result['raw_vs_controller_angle_abs_difference_deg'] = stat(
                np.max(np.abs(sampled - np.rad2deg(self.shaft)), axis=1))
        if self.engaged.any():
            begin = np.flatnonzero(self.engaged)[0]
            result['engagement_time_s'] = float(self.time[begin])
            result['attachment_pad_mm'] = (self.attachment[begin] * 1000).tolist()
            result['modeled_tip_minus_attachment_at_engagement_mm'] = (
                (self.tip[begin] - self.material_attachment[begin]) * 1000).tolist()
            result['attachment_world_at_engagement_m'] = self.material_attachment[begin].tolist()
            result['use_eit_during_contact'] = sorted(set(r['use_eit'] for r, use in zip(self.rows, self.engaged) if use))
            result['attachment_reconstruction_error_mm'] = stat(np.linalg.norm(
                self.material_attachment - self.point, axis=1) * 1000, self.engaged)
            result['virtual_task_pivot_base_m'] = self.virtual_pivot.tolist()
            result['virtual_task_radius_mm'] = self.virtual_radius*1000
            result['virtual_tip_reference_max_error_mm'] = float(np.max(self.virtual_reference_residual_mm))
        return result


def setup_3d(ax, center, radius, title, elevation=24, azimuth=-65):
    ax.set_title(title, loc='left', fontsize=10, pad=2, weight='bold')
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0., center[2] - radius), center[2] + radius)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elevation, azim=azimuth)
    ax.set_xlabel('Base X (m)', fontsize=7, labelpad=0)
    ax.set_ylabel('Base Y (m)', fontsize=7, labelpad=0)
    ax.set_zlabel('Z (m)', fontsize=7, labelpad=0)
    ax.tick_params(labelsize=6, pad=0)
    ax.xaxis.set_major_locator(plt.MaxNLocator(3))
    ax.yaxis.set_major_locator(plt.MaxNLocator(3))
    ax.zaxis.set_major_locator(plt.MaxNLocator(3))


def set_line(line, points):
    line.set_data_3d(*np.asarray(points).T)


class Scene:
    def __init__(self, ax, trial):
        self.ax, self.trial = ax, trial
        setup_3d(ax, trial.pivot + [0., 0., .060], .085,
                 trial.label + ' | pad + attachment + joystick (true scale)',
                 elevation=20, azimuth=-60)
        self.shaft, = ax.plot([], [], [], color=COLORS['shaft'], lw=5, marker='o', ms=5)
        self.attachment, = ax.plot([], [], [], 'o', color=COLORS['attachment'], ms=8)
        self.reference, = ax.plot([], [], [], 'x', color=COLORS['reference'], ms=10, mew=2.5)
        self.task_spring, = ax.plot([], [], [], color='#e11d48', lw=3, zorder=20)
        self.gap, = ax.plot([], [], [], '--', color=COLORS['attachment'], lw=1.8)
        self.normal, = ax.plot([], [], [], color='#475569', lw=1)
        self.force, = ax.plot([], [], [], color='#dc2626', lw=2.5)
        self.slider, = ax.plot([], [], [], 's', color=COLORS['alignment'], ms=4)
        self.alignment, = ax.plot([], [], [], ':', color=COLORS['alignment'], lw=1.5)
        self.disk = Poly3DCollection([], facecolor='#94a3b8', edgecolor='#475569', alpha=.35, linewidth=1.2)
        ax.add_collection3d(self.disk)
        self.pivot, = ax.plot(*trial.pivot[:, None], marker='s', color='#166534', ms=7)
        self.label = ax.text2D(.02, .97, '', transform=ax.transAxes, va='top', fontsize=9)
        self.rim = np.column_stack((.045 * np.cos(np.linspace(0, 2*np.pi, 49)),
                                    .045 * np.sin(np.linspace(0, 2*np.pi, 49)), np.zeros(49)))
        u = np.linspace(0, 1, 100)
        envelope = np.minimum(np.minimum(u/.15, (1-u)/.15), 1.)
        self.task_detail = Line2D(.12+.6*u, .84+.015*np.sin(14*np.pi*u)*envelope,
                                  color='#e11d48', lw=2.5, transform=ax.transAxes)
        ax.add_artist(self.task_detail)
        self.task_endpoints = [Line2D([x], [.84], marker=marker, color=color,
                                      markersize=7, transform=ax.transAxes)
                               for x, marker, color in ((.12, 's', '#db2777'), (.72, 'x', '#f59e0b'))]
        for artist in self.task_endpoints:
            ax.add_artist(artist)
        self.task_detail_label = ax.text2D(.09, .88, '', transform=ax.transAxes, fontsize=9)
        endpoints = ('measured shaft            target' if trial.task_spring_uses_shaft
                     else 'attachment              reference')
        self.task_endpoint_label = ax.text2D(.08, .79, endpoints,
                                            transform=ax.transAxes, fontsize=8)

    def update(self, i):
        t = self.trial
        pad, normal = t.pad[i], t.normal[i]
        set_line(self.shaft, [t.pivot, t.tip[i]])
        self.disk.set_verts([pad + self.rim @ t.pad_rotation[i].T])
        set_line(self.normal, [pad, pad + .012 * normal])
        active = t.engaged[i]
        set_line(self.attachment, [t.material_attachment[i]] if active else np.empty((0, 3)))
        set_line(self.reference, [t.reference[i]] if active else np.empty((0, 3)))
        set_line(self.task_spring, spring_curve(t.spring_start[i], t.reference[i], width=.0008)
                 if active else np.empty((0, 3)))
        for artist in (self.task_detail, *self.task_endpoints, self.task_endpoint_label):
            artist.set_visible(active)
        self.task_detail_label.set_text(
            f'TASK spring detail: {np.linalg.norm(t.reference[i]-t.spring_start[i])*1000:.1f} mm (schematic)'
            if active else '')
        set_line(self.gap, [t.material_attachment[i], t.tip[i]] if active else np.empty((0, 3)))
        set_line(self.force, [t.point[i], t.point[i] + t.force[i] * .0008] if active else np.empty((0, 3)))
        center = np.array([float(t.rows[i].get('alignment_centre_m_' + a, np.nan)) for a in XYZ])
        target = np.array([float(t.rows[i].get('alignment_target_m_' + a, np.nan)) for a in XYZ])
        set_line(self.slider, [center] if active and t.alignment_enabled else np.empty((0, 3)))
        set_line(self.alignment, [center, target] if active and t.alignment_enabled else np.empty((0, 3)))
        status = ' | SLIP DETECTED' if t.rows[i].get('joystick_slipped') == 'True' else ''
        self.label.set_text(f'EIT contact: {t.rows[i]["sensor_contact"]}   shaft fresh: {t.shaft_valid[i]}\n'
                            f'Fault: {t.rows[i]["fault"]}{status}')
        self.label.set_color('#b91c1c' if t.rows[i]['fault'] != 'none' or status else '#111827')


class SpringScene:
    """All four links from slider-frame anchors to shaft-aligned target anchors."""
    def __init__(self, ax, trial):
        self.ax, self.trial = ax, trial
        setup_3d(ax, trial.pivot + [0., 0., .025], trial.alignment_half_span * 1.3,
                 (f'4 alignment springs | a = {trial.alignment_half_span*1000:g} mm'
                  if trial.alignment_enabled else 'Alignment VMC disabled | task spring only'),
                 elevation=25, azimuth=-55)
        # Keep virtual anchors above and below Z=0 visible, including tilted frames.
        ax.set_zlim(trial.pivot[2] - trial.alignment_half_span * .9,
                    trial.pivot[2] + trial.alignment_half_span * .9)
        self.actual_arms = [ax.plot([], [], [], color='#db2777', lw=1.6)[0] for _ in range(4)]
        self.target_arms = [ax.plot([], [], [], '--', color='#0891b2', lw=1.6)[0] for _ in range(4)]
        self.springs = [ax.plot([], [], [], color='#b45309', lw=2.5)[0] for _ in range(4)]
        self.actual, = ax.plot([], [], [], 's', color='#db2777', ms=5)
        self.target, = ax.plot([], [], [], 'o', color='#0891b2', ms=5)
        self.centers, = ax.plot([], [], [], ':', color='#64748b', lw=1)
        self.shaft, = ax.plot([], [], [], color=COLORS['shaft'], lw=3)
        self.disk = Poly3DCollection([], facecolor='#94a3b8', edgecolor='#475569', alpha=.22)
        ax.add_collection3d(self.disk)
        self.rim = np.column_stack((.045*np.cos(np.linspace(0, 2*np.pi, 49)),
                                    .045*np.sin(np.linspace(0, 2*np.pi, 49)), np.zeros(49)))
        self.numbers = [ax.text(0., 0., 0., f'S{j+1}', fontsize=8, color='#92400e',
                               bbox=dict(facecolor='white', edgecolor='none', alpha=.8, pad=.4)) for j in range(4)]
        self.label = ax.text2D(.01, .98, '', transform=ax.transAxes, va='top', fontsize=7)
        ax.text2D(.01, .08, 'Pink squares: slider-frame anchors\nCyan circles: shaft-aligned target anchors\n'
                  'Gold coils: S1-S4 (virtual links; coil width schematic)', transform=ax.transAxes, fontsize=7)

    def update(self, i):
        t = self.trial
        active = (t.alignment_enabled and t.engaged[i] and
                  t.rows[i].get('joystick_mechanism') == 'inward_arc' and t.shaft_valid[i])
        empty = np.empty((0, 3))
        actual, target = t.spring_actual_anchors[i], t.spring_target_anchors[i]
        set_line(self.actual, actual if active else empty)
        set_line(self.target, target if active else empty)
        set_line(self.centers, [t.alignment_center[i], t.alignment_target[i]] if active else empty)
        set_line(self.shaft, [t.pivot, t.tip[i]])
        self.disk.set_verts([t.pad[i] + self.rim @ t.pad_rotation[i].T])
        for j in range(4):
            set_line(self.actual_arms[j], [t.alignment_center[i], actual[j]] if active else empty)
            set_line(self.target_arms[j], [t.alignment_target[i], target[j]] if active else empty)
            set_line(self.springs[j], spring_curve(actual[j], target[j], width=.004) if active else empty)
            mid = (actual[j] + target[j]) / 2
            self.numbers[j].set_position((mid[0], mid[1]))
            self.numbers[j].set_3d_properties(mid[2] + .012)
            self.numbers[j].set_visible(active)
        extension = np.linalg.norm(target - actual, axis=1) * 1000
        task = np.linalg.norm(t.reference[i] - t.spring_start[i]) * 1000
        self.label.set_text('Spring endpoint distances (mm):\n' +
                            '   '.join(f'S{j+1}: {extension[j]:.1f}' for j in range(4)) +
                            f'\nTask spring: {task:.1f} mm' if active else
                            ('Alignment disabled; task spring active' if t.engaged[i] and not t.alignment_enabled
                             else f'{t.modes[i].upper()}: alignment springs inactive'))


def render(trials, output, fps, speed, rotation_source, preview=False):
    count = len(trials)
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='white')
    if count == 2:
        # Put the conditions side by side so each mechanism gets most of the
        # available height. The alignment view is kept only when it is active.
        gs = GridSpec(5, 4, figure=fig, width_ratios=(1.7, .8, 1.7, .8),
                      height_ratios=(2.8, 2.8, 1., 1., 1.),
                      left=.025, right=.98, top=.91, bottom=.115, hspace=.36, wspace=.18)
    else:
        gs = GridSpec(3, 3, figure=fig, width_ratios=(1.6, .8, 1.2),
                      left=.025, right=.98, top=.91, bottom=.105, hspace=.32, wspace=.18)
    block_names = ' / '.join(dict.fromkeys(t.block.name for t in trials))
    fig.suptitle(output.stem.replace('_', ' ') + ' | ' + block_names,
                 fontsize=15, weight='bold', x=.025, ha='left', y=.98)
    fig.text(.025, .937, 'Telemetry reconstruction | pink coil: task spring | red arrow: requested force (0.8 mm/N) | '
             f'playback {speed:g}x', fontsize=10, color='#475569')
    scenes, cursors, diagnostics = [], [], []
    for index, tr in enumerate(trials):
        if count == 2:
            col = 2 * index
            pad_cell = gs[:2, col] if tr.alignment_enabled else gs[:2, col:col+2]
            alignment_cell = gs[:2, col+1] if tr.alignment_enabled else None
            chart_cells = [gs[row, col:col+2] for row in (2, 3, 4)]
        else:
            pad_cell = gs[:, 0] if tr.alignment_enabled else gs[:, :2]
            alignment_cell = gs[:, 1] if tr.alignment_enabled else None
            chart_cells = [gs[row, 2] for row in range(3)]
        scenes.append((Scene(fig.add_subplot(pad_cell, projection='3d'), tr), tr))
        if alignment_cell is not None:
            scenes.append((SpringScene(fig.add_subplot(alignment_cell, projection='3d'), tr), tr))
        angle = fig.add_subplot(chart_cells[0])
        align = fig.add_subplot(chart_cells[1], sharex=angle)
        force = fig.add_subplot(chart_cells[2], sharex=angle)
        selected = 1 if tr.axis == 'y' else 0
        measured = np.rad2deg(tr.shaft)
        measured[~tr.shaft_valid] = np.nan
        command = np.where(tr.engaged, np.rad2deg(tr.command[:, selected]), np.nan)
        angle.plot(tr.time, command, '--', color='#f59e0b', lw=1.2, label=f'Command {tr.axis.upper()}')
        angle.plot(tr.time, measured[:, selected], color='#16a34a', lw=1.2, label='Measured')
        angle.plot(tr.time, measured[:, 1-selected], color='#64748b', lw=.8, alpha=.7, label='Other axis')
        if tr.raw_angles:
            raw_t, raw_angles = tr.raw_angles
            angle.plot(raw_t, raw_angles[:, selected], color='#22c55e', lw=.5, alpha=.3, label='Raw ESP32')
        angle.set_ylabel('Tilt (deg)', fontsize=8)
        angle.legend(loc='upper left', fontsize=6, ncol=4)
        align.plot(tr.time, tr.alignment_error, color='#7c3aed', label='Pad/shaft error (deg)')
        align.plot(tr.time, tr.normal_gap_mm, color='#db2777', alpha=.8, label='Modeled plane gap (mm)')
        align.axhline(0, color='#cbd5e1', lw=.7)
        align.set_ylabel('deg / mm', fontsize=8)
        align.legend(loc='upper left', fontsize=6, ncol=2)
        force.plot(tr.time, np.linalg.norm(tr.force, axis=1), color='#dc2626', label='Requested force norm (N)')
        force.plot(tr.time, tr.flag('sensor_contact').astype(float) * 5, color='#0891b2', lw=1, label='EIT contact x5')
        force.set_ylabel('N / contact', fontsize=8)
        force.set_xlabel('Seconds from start of this trace', fontsize=8)
        force.legend(loc='upper left', fontsize=6, ncol=2)
        for ax in (angle, align, force):
            ax.set_xlim(0, max(t.time[-1] for t in trials))
            ax.grid(alpha=.2)
            ax.tick_params(labelsize=7)
            ax.margins(y=.2)
            cursors.append((ax.axvline(0, color='#0f172a', lw=1), tr))
            for change in tr.metrics()['mode_changes'][1:]:
                ax.axvline(change['time_s'], color='#94a3b8', alpha=.25, lw=.6)
            fault = tr.metrics()['first_fault']
            if fault:
                ax.axvline(fault['time_s'], color='red', lw=1, ls='--')
        diagnostics.append((fig.text(.025 + (.49 * index if count == 2 else 0.),
                                     .053 if count == 2 else .049, '',
                                     fontsize=8, color='#334155'), tr))
    fig.text(.025, .015, 'Tip/pivot use recorded L and pivot settings; gap is a model check, not measured separation. '
             'No camera footage or contact-force sensor. Replay stops at last saved sample.', fontsize=8, color='#64748b')
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'avc1'), fps, (1920, 1080))
    if not writer.isOpened():
        plt.close(fig)
        raise RuntimeError('OpenCV H.264 encoder unavailable')
    duration = max(t.time[-1] for t in trials)
    interaction_midpoints = [float(np.median(t.time[t.modes == 'interact']))
                             for t in trials if np.any(t.modes == 'interact')]
    poster_at = float(np.median(interaction_midpoints)) if interaction_midpoints else .5 * duration
    frames = 1 if preview else int(np.ceil((duration / speed + 1.5) * fps))
    poster = output.with_suffix('.png')
    poster_frame = 0 if preview else min(frames - 1, int(poster_at / speed * fps))
    try:
        for frame in range(frames):
            at = poster_at if preview else min(frame / fps * speed, duration)
            for scene, tr in scenes:
                i = min(np.searchsorted(tr.time, at, side='right') - 1, len(tr.rows) - 1)
                scene.update(max(0, i))
            for line, tr in cursors:
                line.set_xdata([min(at, tr.time[-1])] * 2)
            for label, tr in diagnostics:
                i = min(np.searchsorted(tr.time, at, side='right') - 1, len(tr.rows) - 1)
                row = tr.rows[i]
                ended = ' | RECORD ENDED' if at >= tr.time[-1] else ''
                label.set_text(f'{tr.label} | {tr.modes[i]}{ended} | '
                               f'attachment ({tr.attachment[i,0]*1000:+.1f}, '
                               f'{tr.attachment[i,1]*1000:+.1f}) mm\n'
                               f'pad/shaft {tr.alignment_error[i]:.1f}° | '
                               f'plane gap {tr.normal_gap_mm[i]:+.1f} mm | '
                               f'force {np.linalg.norm(tr.force[i]):.1f} N | fault: {row["fault"]}' +
                               (' | SLIP DETECTED' if row.get('joystick_slipped') == 'True' else ''))
            fig.canvas.draw()
            rgba = np.asarray(fig.canvas.buffer_rgba())
            writer.write(cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR))
            if frame == poster_frame:
                cv2.imwrite(str(poster), cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR))
            if frame % (fps * 5) == 0:
                print(f'  {output.name}: frame {frame}/{frames}', flush=True)
    finally:
        writer.release()
        plt.close(fig)
    capture = cv2.VideoCapture(str(output))
    decoded, _ = capture.read()
    actual_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    capture.release()
    if not decoded or actual_frames != frames:
        raise RuntimeError(f'Video validation failed for {output}: {actual_frames}/{frames}')
    return dict(file=output.name, poster=poster.name, frames=frames, fps=fps,
                playback_speed=speed, source_traces=[str(t.path) for t in trials])


def write_report(output, metrics, videos, blocks):
    data = dict(generated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                blocks=blocks, trials=metrics, videos=videos,
                caveats=['Host receipt times do not measure acquisition/USB transport delay.',
                         'Robot uses nominal FR3 kinematics; fixed tool offset is fitted and residual reported.',
                         'Pad normal, center and attachment during engagement are recorded telemetry.',
                         'Four alignment anchor pairs follow slider_alignment: centre + R*(+/-a,0,0) or R*(0,+/-a,0).',
                         'Target-frame yaw is inferred from the first approach pose; desired normal is logged. Coil width is schematic.',
                         'Shaft length and pivot are controller settings, not independently measured geometry.',
                         'Pad mount outline and robot links are schematic; no collision proof or camera footage.',
                         'Speed derived from 50 Hz host samples cannot establish 1 kHz fault peak or its cause.',
                         'Done reports controller completion, not externally verified joystick success.'])
    (output / 'diagnostics.json').write_text(json.dumps(data, indent=2) + '\n')
    items = []
    for m in metrics:
        def fmt(value):
            return '-' if value is None else f'{value:.1f}'
        angle = m['interact_angle_rmse_deg']
        align = m['alignment_error_deg']
        gap = m['modeled_tip_to_pad_plane_mm']
        contact = m['contact_false_fraction']
        items.append('<tr>' + ''.join(f'<td>{html.escape(str(v))}</td>' for v in (
            m['block'].split('_2026')[0] + ' ' + m['block'].split('2026-10-25_')[-1][:8],
            m['condition'], ('joystick slip' if m.get('joystick_slipped') else
                             'contact release' if m.get('trial_contact_released') else m['final_mode']),
            (m['first_fault'] or {}).get('name', 'none'),
            fmt(angle), fmt(align['max'] if align else None),
            fmt(gap['median'] if gap else None), fmt(contact * 100 if contact is not None else None))) + '</tr>')
    findings = []
    latest = [m for m in metrics if m['condition'] in ('baseline', 'EIT', 'kinematic')]
    if latest:
        newest_block = latest[0]['block']
        newest_pair = [m for m in latest if m['block'] == newest_block]
        for m in newest_pair:
            phase = m['command_phase_angles_deg']
            negative = phase['negative_hold']
            findings.append(f'{m["block"]}, {m["condition"]}: angle RMSE '
                            f'{m["interact_angle_rmse_deg"]:.1f}°, '
                            f'{m["shaft_clipped_fraction"]*100:.1f}% of interaction samples flagged clipped; '
                            + (f'median measured tilt {negative["median"]:.2f}° while command < −18°.'
                               if negative else 'no recorded interaction samples with command < −18°.'))
            if m['condition'] == 'EIT' and 'modeled_tip_minus_attachment_at_engagement_mm' in m:
                mismatch = np.array(m['modeled_tip_minus_attachment_at_engagement_mm'])
                findings.append(f'{m["block"]}, EIT: modeled tip versus recorded attachment differs by '
                                f'{np.linalg.norm(mismatch):.1f} mm at engagement '
                                f'(Base XYZ: {mismatch[0]:+.1f}, {mismatch[1]:+.1f}, {mismatch[2]:+.1f} mm). '
                                'This depends on the configured pivot/length and EIT coordinate calibration; '
                                'check these before attributing it to sliding or contact loss.')
    for m in latest:
        if m.get('joystick_slipped'):
            findings.append(f'{m["block"]}, {m["condition"]}: the controller detected joystick slip '
                            'and retracted; this trial is marked failed.')
        elif m.get('trial_contact_released'):
            findings.append(f'{m["block"]}, {m["condition"]}: the controller recorded trial_contact_released=True; '
                            'treat this as an unsuccessful release even though the final mode is done.')
    findings_html = ''.join('<li>' + html.escape(f) + '</li>' for f in findings)
    cards = ''.join(f'<article><h2>{html.escape(v["file"].replace("_", " "))}</h2>'
                    f'<video controls preload="metadata" poster="{v["poster"]}" src="{v["file"]}"></video>'
                    f'<p><a href="{v["file"]}">Open / download MP4</a> · {v["playback_speed"]:g}× playback</p>'
                    f'<details><summary>Source traces</summary><pre>{html.escape(chr(10).join(v["source_traces"]))}</pre></details>'
                    '</article>' for v in videos)
    document = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Recent EIT joystick replays</title><style>body{font:16px system-ui;background:#f1f5f9;color:#0f172a;max-width:1400px;margin:32px auto;padding:0 24px;line-height:1.55}article,section{background:white;padding:24px;margin:24px 0;border-radius:12px}video{width:100%;background:white}table{border-collapse:collapse;font-size:13px;width:100%}th,td{padding:8px;text-align:left;border-bottom:1px solid #ddd}pre{white-space:pre-wrap;font-size:12px}h2{font-size:20px}a{color:#2563eb}</style>
<h1>Recent EIT joystick replays</h1><p>Offline reconstruction of saved records. All geometry is in robot Base coordinates.
The paired videos start at each trace's own start time; these conditions were run sequentially.</p>
<section><h2>How to read the replay</h2><p>Gray: 90 mm pad.
Green: joystick modeled from recorded angles and configured pivot/length. Pink: logged controller attachment (filtered moving contact when enabled; otherwise fixed).
Orange: virtual task reference. Purple: virtual slider and alignment target when enabled. Red: requested force (0.8 mm/N).</p>
<p><b>Spring views:</b> The close-up shows the pink task spring between the recorded palm attachment and its reference, or between the measured shaft point and target when shaft mode is active.
The separate wide view shows all four gold alignment springs S1–S4, pink square anchors on the common palm slider,
and cyan circle anchors on the shaft-aligned target frame. The actual anchor half-span is 160 mm in these runs,
so the four alignment springs extend beyond the physical 90 mm pad. These are virtual controller links and are hidden when alignment VMC is disabled.
Coil widths are drawn for visibility; endpoints and frame spacing retain their spatial scale.</p>
<p>The enlarged pad and joystick view uses true spatial scale. Positive modeled plane gap means the modeled tip lies on the active-face side of the pad plane.
A gap or lateral disagreement can identify a calibration/model inconsistency; it does not establish physical separation.</p>
<p>Angle, contact and force charts retain all saved samples. Animation uses the most recent sample at each frame; it does not smooth across faults.
Raw ESP32 angles use original host receipt timestamps and copied calibration, with no fitted time shift.</p></section>
<section><h2>Measured findings</h2><ul>''' + findings_html + '''</ul></section>
<section><h2>Findings to investigate</h2><ul>
<li>The 23:17:51 block was interrupted during Ready; it contains no manipulation trial. Newer completed blocks are included when present.</li>
<li>The y_plus/y_minus names describe Y landing offsets. Their recorded command axis is X.</li>
<li>Recent completed records still show measured-versus-command angle error and intervals without EIT contact.
Controller completion alone is not evidence of object success.</li>
<li>Earlier nearby attempts record joint_speed faults. The replay shows the saved lead-up; joint speeds estimated at host sample rate cannot identify the unsaved servo-rate peak or establish cause.</li>
<li>Compare the logged attachment against the green modeled tip. Their discrepancy depends on pivot, length, pad transform and EIT inference.
Confirm those geometries before interpreting a gap as contact loss.</li></ul></section>
<section><h2>Interaction-only diagnostics</h2><p>Fresh shaft samples in Interact only. See <a href="diagnostics.json">diagnostics.json</a> for exact paths,
mode timestamps, kinematic residuals and full metrics. EIT-contact false fraction is an electrical classification.</p><table><thead><tr>
<th>Block / time JST</th><th>Condition</th><th>Final mode</th><th>Fault</th><th>Angle RMSE °</th><th>Max alignment °</th><th>Median modeled gap mm</th><th>No EIT contact %</th>
</tr></thead><tbody>''' + ''.join(items) + '</tbody></table></section>' + cards
    document += '<section><h2>Limits</h2><ul>' + ''.join('<li>' + html.escape(c) + '</li>' for c in data['caveats']) + '</ul></section></html>'
    (output / 'index.html').write_text(document)


def render_missing_samples(block, output, skipped, fps=8, preview=False):
    """Explain an aborted block when there are no samples to reconstruct."""
    result_path = block / 'block_result.json'
    result = json.loads(result_path.read_text()) if result_path.is_file() else {}
    reason = result.get('error', 'The controller trace has fewer than two complete samples.')
    snapshots = sorted((block / 'reproducibility' / 'snapshots').glob('*.before_motion.json'))
    if snapshots:
        snapshot = json.loads(snapshots[-1].read_text())
        reason = snapshot.get('error') or reason
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='white')
    fig.text(.06, .83, 'Joystick VMC replay unavailable', fontsize=32,
             weight='bold', color='#0f172a')
    fig.text(.06, .70, 'No usable controller samples were saved. Robot motion cannot be reconstructed.',
             fontsize=17, color='#334155')
    details = (f'Block: {block.name}\nStage: {result.get("stage", "unknown")}\n'
               f'Status: {result.get("status", "unknown")}\n'
               f'Cause: {reason}\n'
               f'Short traces: {len(skipped)}')
    fig.text(.06, .59, '\n'.join(textwrap.wrap(details, 105, replace_whitespace=False,
                                               break_long_words=False)),
             fontsize=14, color='#475569', va='top', linespacing=1.7)
    fig.canvas.draw()
    frame = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()), cv2.COLOR_RGBA2BGR)
    plt.close(fig)
    poster = output / 'joystick_vmc_summary.png'
    video = output / 'joystick_vmc_replay.mp4'
    cv2.imwrite(str(poster), frame)
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'avc1'), fps,
                             (frame.shape[1], frame.shape[0]))
    if not writer.isOpened():
        raise RuntimeError('OpenCV H.264 encoder unavailable')
    try:
        for _ in range(1 if preview else 2 * fps):
            writer.write(frame)
    finally:
        writer.release()
    report = dict(block=str(block), generated_utc=datetime.datetime.now(
        datetime.timezone.utc).isoformat(), status='no_controller_samples',
        note='No robot motion is inferred from these traces.',
        skipped_traces=skipped, block_result=result, diagnostic_reason=reason,
        videos=[dict(relative_path=video.name)])
    (output / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'index.html').write_text(
        '<!doctype html><html lang="en"><meta charset="utf-8">'
        '<title>Joystick VMC replay unavailable</title>'
        '<style>body{font:16px system-ui;max-width:1200px;margin:auto;padding:24px}'
        'video{width:100%}</style><h1>Joystick VMC replay unavailable</h1>'
        '<p>No usable controller samples were saved. Robot motion cannot be reconstructed.</p>'
        f'<p>Cause: {html.escape(reason)}</p>'
        f'<video controls poster="{poster.name}" src="{video.name}"></video></html>')
    print(f'Joystick VMC diagnostic: {output}', flush=True)


def render_block(block, output, kinematics, fps=8, speed=2., preview=False):
    """Render every available manipulation trace, including faulted trials."""
    output.mkdir(parents=True, exist_ok=True)
    paths = sorted(block.glob('repeat_*/joystick_*.csv'))
    if not paths:
        raise FileNotFoundError(f'No joystick controller traces in {block}')
    counts = {path: len(rows_from(path)) for path in paths}
    skipped = [dict(path=str(path), complete_samples=count)
               for path, count in counts.items() if count < 2]
    usable = [path for path, count in counts.items() if count >= 2]
    if not usable:
        render_missing_samples(block, output, skipped, fps, preview)
        return
    trials = [Trial(path, kinematics) for path in usable]
    rotation, rotation_source = fit_pad_rotation(trials)
    for trial in trials:
        trial.finish_geometry(rotation, rotation_source)
    manipulation = [trial for trial in trials if trial.condition != 'ready']
    groups = []
    for trial in manipulation:
        groups.append(([trial], output / trial.path.parent.name /
                       (trial.path.stem + '_vmc.mp4')))
    for repeat in sorted({trial.path.parent.name for trial in manipulation}):
        pair = [trial for trial in manipulation if trial.path.parent.name == repeat and
                trial.condition in ('baseline', 'EIT')]
        if {trial.condition for trial in pair} == {'baseline', 'EIT'}:
            pair = [next(trial for trial in pair if trial.condition == condition)
                    for condition in ('baseline', 'EIT')]
            groups.append((pair, output / repeat / 'joystick_vmc_pair.mp4'))
    if not groups:
        latest = max(trials, key=lambda trial: trial.ns[0])
        groups = [([latest], output / latest.path.parent.name /
                   (latest.path.stem + '_vmc.mp4'))]
    if preview:
        groups = [next((group for group in groups if len(group[0]) == 2), groups[0])]
    videos = []
    for selected, path in groups:
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f'Rendering {path}', flush=True)
        result = render(selected, path, fps, speed, rotation_source, preview)
        result['relative_path'] = str(path.relative_to(output))
        videos.append(result)
    if not preview:
        latest = next((video for video in reversed(videos)
                       if video['file'] == 'joystick_vmc_pair.mp4'), videos[-1])
        source = output / latest['relative_path']
        shutil.copy2(source, output / 'joystick_vmc_replay.mp4')
        shutil.copy2(source.with_suffix('.png'), output / 'joystick_vmc_summary.png')
    report = dict(block=str(block), generated_utc=datetime.datetime.now(
        datetime.timezone.utc).isoformat(),
        notes=['Offline robot and ESP32 telemetry reconstruction; no camera footage.',
               'ESP32 angles at an ADC rail are bounded calibration outputs, not measured angles beyond the rail.',
               'Physical joystick tip uses the recorded pivot, shaft length and measured angles.',
               'In shaft mode, the pink coil shows XY shaft-angle error; the same task spring also holds palm contact height in Z.',
               'The four gold alignment springs are virtual links and are hidden when disabled; their coil width is schematic.'],
        trials=[trial.metrics() for trial in trials], videos=videos,
        skipped_traces=skipped)
    (output / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
    cards = ''.join(
        f'<article><h2>{html.escape(video["relative_path"])}</h2>'
        f'<video controls preload="metadata" poster="{html.escape(str(Path(video["relative_path"]).with_suffix(".png")))}" '
        f'src="{html.escape(video["relative_path"])}"></video></article>'
        for video in videos)
    (output / 'index.html').write_text(
        '<!doctype html><html lang="en"><meta charset="utf-8">'
        '<title>Joystick VMC replay</title><style>body{font:16px system-ui;max-width:1400px;'
        'margin:auto;padding:24px;background:#f1f5f9}article{background:white;padding:20px;'
        'margin:20px 0}video{width:100%}</style><h1>Joystick VMC replay</h1>'
        '<p>Offline reconstruction from robot and ESP32 records. No camera footage. '
        'ESP32 values at ±35° are clipped calibration estimates.</p>' + cards + '</html>')
    print(f'Joystick VMC videos: {output}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--kinematics', type=Path,
                        help='FR3 kinematics YAML; defaults to the installed franka_description package')
    parser.add_argument('--recent-blocks', type=int, default=10)
    parser.add_argument('--fps', type=int, default=12)
    parser.add_argument('--speed', type=float, default=2., help='Playback speed for a single block')
    parser.add_argument('--analyze-only', action='store_true')
    parser.add_argument('--preview', action='store_true', help='Render one review frame from the first video')
    parser.add_argument('--refresh-report', action='store_true', help='Recompute diagnostics, preserving existing video manifest')
    parser.add_argument('--replay-manifest', type=Path, help='Replay exactly the blocks and video groups from a previous diagnostics.json')
    parser.add_argument('--last-trial', action='store_true', help='Render only the latest recorded manipulation trial')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.kinematics is None:
        from ament_index_python.packages import get_package_share_directory
        args.kinematics = (Path(get_package_share_directory('franka_description')) /
                           'robots/fr3/kinematics.yaml')
    kinematics = yaml.safe_load(args.kinematics.read_text())
    if (args.root / 'block.json').is_file():
        if args.fps <= 0 or args.speed <= 0:
            parser.error('--fps and --speed must be positive')
        render_block(args.root, args.output, kinematics, args.fps, args.speed, args.preview)
        return
    blocks = sorted((p for p in args.root.iterdir() if (p / 'block.json').exists()),
                    key=lambda p: json.loads((p / 'block.json').read_text()).get('created_utc', ''), reverse=True)[:args.recent_blocks]
    previous_report = args.output / 'diagnostics.json'
    if args.refresh_report and previous_report.exists():
        blocks = [Path(b['path']) for b in json.loads(previous_report.read_text())['blocks']]
    replay_manifest = json.loads(args.replay_manifest.read_text()) if args.replay_manifest else None
    if replay_manifest:
        blocks = [Path(b['path']) for b in replay_manifest['blocks']]
    trials, block_info = [], []
    for block in blocks:
        result_path = block / 'block_result.json'
        block_info.append(dict(path=str(block), result=json.loads(result_path.read_text()) if result_path.exists() else None))
        for path in sorted(block.glob('repeat_*/*.csv')):
            trial = Trial(path, kinematics)
            trials.append(trial)
    if not trials:
        raise SystemExit('No trial CSVs found')
    rotation, rotation_source = fit_pad_rotation(trials)
    for trial in trials:
        trial.finish_geometry(rotation, rotation_source)
    metrics = [t.metrics() for t in trials]
    videos = (json.loads(previous_report.read_text()).get('videos', [])
              if args.refresh_report and previous_report.exists() else [])
    write_report(args.output, metrics, videos, block_info)
    for m in metrics:
        if m['condition'] != 'ready':
            print(json.dumps({k:m[k] for k in ('block','condition','final_mode','first_fault','interact_angle_rmse_deg',
                                               'alignment_error_deg','modeled_tip_to_pad_plane_mm','contact_false_fraction',
                                               'fk_position_residual_mm')}), flush=True)
    if args.analyze_only or args.refresh_report:
        return
    groups = []
    pairs = 0
    for block in blocks:
        pair = [t for t in trials if t.block == block and t.condition in ('baseline','EIT') and t.modes[-1] == 'done']
        if {t.condition for t in pair} == {'baseline', 'EIT'} and (pairs < 2 or
                not any(g[0][0].block.name.startswith('y_minus_') for g in groups) and block.name.startswith('y_minus_')):
            pair = [next(t for t in pair if t.condition == c) for c in ('baseline', 'EIT')]
            groups.append((pair, f'{pairs+1:02d}_{block.name.split("_2026")[0]}_latest_pair.mp4', 1.))
            pairs += 1
    faults = [t for t in trials if t.condition != 'ready' and any(r['fault'] != 'none' for r in t.rows)]
    if faults:
        groups.append((faults[:2], '03_recent_joint_speed_faults.mp4', .5))
    ready_only = next(([t for t in trials if t.block == block] for block in blocks
                       if any(t.block == block for t in trials)
                       and not any(t.block == block and t.condition != 'ready' for t in trials)), [])
    if ready_only:
        groups.append(([ready_only[-1]], '04_interrupted_ready.mp4', 1.))
    unfinished = [t for t in trials if t.condition != 'ready' and t.modes[-1] == 'interact'
                  and all(r['fault'] == 'none' for r in t.rows)]
    if unfinished:
        groups.append(([unfinished[0]], '05_unfinished_interaction.mp4', 2.))
    if replay_manifest:
        by_path = {str(t.path): t for t in trials}
        groups = [([by_path[p] for p in v['source_traces']],
                   Path(v['file']).stem + '_springs.mp4', v['playback_speed'])
                  for v in replay_manifest['videos']]
    if args.last_trial:
        last = max((t for t in trials if t.condition != 'ready'), key=lambda t: t.ns[0])
        groups = [([last], f'last_experiment_{last.condition}_springs.mp4', 1.)]
    for group, name, speed in groups:
        print(f'Rendering {name}', flush=True)
        video = render(group, args.output / name, args.fps, speed, rotation_source, args.preview)
        videos.append(video)
        write_report(args.output, metrics, videos, block_info)
        if args.preview:
            break
    print(f'Report: {args.output / "index.html"}', flush=True)


if __name__ == '__main__':
    main()
