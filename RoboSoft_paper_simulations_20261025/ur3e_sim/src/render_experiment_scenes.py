"""Render representative robot states from saved S1-S4 MuJoCo trials."""
from pathlib import Path
import csv
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from robot import ik, make_model


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results'
FIG = ROOT / 'report' / 'figures'
P0 = np.array([.3, 0., .25])
R0 = np.diag([1., -1., -1.])


def trial(**criteria):
    with (OUT / 'trial_metrics.csv').open(newline='') as file:
        for row in csv.DictReader(file):
            if all(str(row[key]) == str(value) for key, value in criteria.items()):
                trace = np.load(OUT / 'trial_data' / f"trial_{int(row['trial_id']):04d}.npz")['trace']
                return row, trace
    raise ValueError(f'No saved trial matches {criteria}')


def frame(row, trace, elapsed, zoom=False):
    index = np.argmin(abs(trace[:, 0] - (float(row['latch_time_s']) + elapsed)))
    sample = trace[index]
    spring = float(row['spring']) if row['spring'] else 0.
    if np.isnan(spring):
        spring = 0.
    model, xml = make_model(spring=spring)
    root = ET.fromstring(xml)
    base = root.find(".//body[@name='base']")
    ET.SubElement(base, 'geom', type='capsule', size='.034',
                  fromto='0 0 .025 0 0 .15185', rgba='.7 .73 .76 1',
                  contype='0', conaffinity='0')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', type='skybox', builtin='flat',
                  rgb1='.96 .97 .98', rgb2='.96 .97 .98', width='128', height='128')
    visual = root.find('visual')
    ET.SubElement(visual, 'headlight', ambient='.6 .6 .6',
                  diffuse='.8 .8 .8', specular='.2 .2 .2')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
    data = mujoco.MjData(model)
    data.qpos[:] = ik(model, P0, R0)
    data.qpos[:6] = sample[12:18]
    if model.nq == 7:
        data.qpos[6] = sample[9]
    offset = float(row['offset_mm']) / 1000
    model.body_pos[model.body('support').id] = P0 + R0 @ [offset, 0, 0] + [0, 0, -.009]
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.3, 0, .24] if zoom else [.17, 0, .24]
    camera.distance = .34 if zoom else .95
    camera.azimuth = 100
    camera.elevation = -12
    renderer = mujoco.Renderer(model, height=600, width=800)
    renderer.update_scene(data, camera)
    image = renderer.render().copy()
    renderer.close()
    return image, sample


def save_scene(name, panels):
    fig, axes = plt.subplots(1, 3, figsize=(11.2, 3.3))
    for ax, (row, trace, elapsed, label, zoom) in zip(axes, panels):
        image, sample = frame(row, trace, elapsed, zoom)
        ax.imshow(image)
        ax.set_title(label, fontsize=10, color='#17344b')
        ax.text(.5, -.035,
                f"trial {int(row['trial_id'])}  |  t={sample[0]:.2f} s  |  "
                f"F={sample[1]:.1f} N  |  angle={np.degrees(sample[3]):+.1f}°",
                transform=ax.transAxes, ha='center', va='top', fontsize=7.5)
        ax.axis('off')
    fig.subplots_adjust(left=.01, right=.99, top=.88, bottom=.13, wspace=.025)
    fig.savefig(FIG / f'{name}.png', dpi=190, facecolor='white')
    fig.savefig(FIG / f'{name}.pdf', facecolor='white')
    plt.close(fig)


def main():
    s1 = trial(stage='S1', condition='eit', offset_mm='30.0', seed='700')
    save_scene('scene_s1', [(s1[0], s1[1], -.3, 'Spring support: approach', False),
                            (s1[0], s1[1], 2.3, 'Anchor loads support', True),
                            (s1[0], s1[1], 4.1, 'Anchor returns', True)])
    s2 = trial(stage='S2', condition='eit', offset_mm='30.0', angle_deg='5', seed='710')
    save_scene('scene_s2', [(s2[0], s2[1], -.2, 'Fixed support: approach', False),
                            (s2[0], s2[1], 2.3, 'Supported rotation', True),
                            (s2[0], s2[1], 4.1, 'Return', True)])
    fast = trial(stage='S3', condition='eit', offset_mm='27.0', angle_deg='5',
                 seed='720', rate_hz='20.0', coarse_mm='0.0', delay_ms='0.0', noise='0.0')
    slow = trial(stage='S3', condition='eit', offset_mm='27.0', angle_deg='5',
                 seed='720', rate_hz='1.0', coarse_mm='0.0', delay_ms='0.0', noise='0.0')
    save_scene('scene_s3', [(slow[0], slow[1], -.5, '1 Hz: still approaching', False),
                            (fast[0], fast[1], .1, '20 Hz: acquired', True),
                            (slow[0], slow[1], .1, '1 Hz: acquired', True)])
    soft = trial(stage='S4', condition='force_eit', spring='500.0',
                 target_N='4.0', seed='740')
    stiff = trial(stage='S4', condition='force_eit', spring='1500.0',
                  target_N='4.0', seed='740')
    save_scene('scene_s4', [(soft[0], soft[1], -.3, 'Spring support: approach', False),
                            (soft[0], soft[1], 2.3, '500 N/m support', True),
                            (stiff[0], stiff[1], 2.3, '1500 N/m support', True)])


if __name__ == '__main__':
    main()
