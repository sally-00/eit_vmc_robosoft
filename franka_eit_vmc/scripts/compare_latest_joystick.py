#!/usr/bin/env python3
"""Large, contact-aligned comparison of every condition in the newest joystick block."""
import argparse
import datetime
import html
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from replay_joystick import Trial, fit_pad_rotation, set_line, spring_curve, SpringScene


EMPTY = np.empty((0, 3))
PINK, GREEN, ORANGE, PURPLE, BLUE = '#db2777', '#15803d', '#ea580c', '#7c3aed', '#2563eb'
NAMES = {'baseline': 'BASELINE — pad center', 'EIT': 'EIT — detected contact',
         'kinematic': 'KINEMATIC — projected tip'}


class LargePad:
    def __init__(self, ax, trial, center):
        self.ax, self.t = ax, trial
        # Common, tightly cropped physical scale across all cases. No robot/mount.
        self.origin = trial.pivot
        c = (center - self.origin) * 1000
        ax.set(xlim=(c[0]-54, c[0]+54), ylim=(c[1]-54, c[1]+54),
               zlim=(c[2]-30, c[2]+30))
        ax.set_box_aspect((1, 1, .6), zoom=1.21)
        ax.view_init(elev=34, azim=-65)
        ax.set_title('Palm + measured / virtual joystick', fontsize=17, weight='bold', pad=2)
        ax.set_xlabel('Base X from pivot (mm)', fontsize=12, labelpad=3)
        ax.set_ylabel('Base Y from pivot (mm)', fontsize=12, labelpad=3)
        ax.set_zlabel('Z (mm)', fontsize=12, labelpad=1)
        ax.tick_params(labelsize=11, pad=0)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_major_locator(plt.MaxNLocator(3))
        self.rim = np.column_stack((.045*np.cos(np.linspace(0, 2*np.pi, 90)),
                                    .045*np.sin(np.linspace(0, 2*np.pi, 90)), np.zeros(90)))
        self.disk = Poly3DCollection([], facecolor='#cbd5e1', edgecolor='#475569',
                                     alpha=.16, linewidth=2, zorder=1)
        ax.add_collection3d(self.disk)
        self.rim_line, = ax.plot([], [], [], color='#64748b', lw=2, zorder=2)
        self.center, = ax.plot([], [], [], '+', color='#334155', ms=13, mew=3, zorder=10)
        self.shaft, = ax.plot([], [], [], color=GREEN, lw=8, marker='o', ms=8, zorder=5)
        self.virtual_shaft, = ax.plot([], [], [], '--', color=BLUE, lw=6, zorder=15)
        self.virtual_pivot, = ax.plot([], [], [], 'D', color=BLUE, ms=11, mec='white', mew=1.5, zorder=16)
        self.contact, = ax.plot([], [], [], 'o', color=PINK, ms=17, mec='white', mew=2.5, zorder=25)
        self.slider, = ax.plot([], [], [], 's', color=PURPLE, ms=12, mec='white', mew=1.5, zorder=20)
        self.reference, = ax.plot([], [], [], 'x', color=ORANGE, ms=17, mew=4, zorder=30)
        self.radial, = ax.plot([], [], [], '--', color=PINK, lw=2.2, zorder=8)
        self.spring, = ax.plot([], [], [], color=PINK, lw=4, zorder=22)
        self.name = ax.text2D(.03, .92, '', transform=ax.transAxes, fontsize=13, color=PINK,
                              bbox=dict(facecolor='white', alpha=.9, edgecolor='none', pad=2))

    def update(self, i):
        t = self.t
        convert = lambda p: (np.asarray(p) - self.origin) * 1000
        pad, attachment = t.pad[i], t.material_attachment[i]
        rim = pad + self.rim @ t.pad_rotation[i].T
        self.disk.set_verts([convert(rim)])
        set_line(self.rim_line, convert(rim))
        set_line(self.center, convert([pad]))
        set_line(self.shaft, convert([t.pivot, t.tip[i]]))
        set_line(self.contact, convert([attachment]))
        set_line(self.radial, convert([pad, attachment]))
        active = t.engaged[i]
        set_line(self.virtual_shaft,convert([t.virtual_pivot,t.virtual_tip[i]]) if active else EMPTY)
        set_line(self.virtual_pivot,convert([t.virtual_pivot]) if active else EMPTY)
        set_line(self.slider, convert([t.alignment_center[i]]) if active else EMPTY)
        set_line(self.reference, convert([t.reference[i]]) if active else EMPTY)
        set_line(self.spring, convert(spring_curve(attachment, t.reference[i], width=.0013, turns=5)) if active else EMPTY)
        self.name.set_text('PINK palm attachment -> spring -> ORANGE virtual tip\n'
                           f'BLUE: virtual joystick ({t.virtual_radius*1000:g} mm); GREEN: measured')


class PadFace:
    def __init__(self, ax, trial):
        self.ax, self.t = ax, trial
        ax.set(xlim=(-53, 53), ylim=(-53, 53), aspect='equal')
        ax.add_patch(Circle((0, 0), 45, fc='#f8fafc', ec='#475569', lw=2))
        ax.axhline(0, color='#cbd5e1', lw=.8)
        ax.axvline(0, color='#cbd5e1', lw=.8)
        ax.set_title('Face-on pad coordinates — same scale', fontsize=17, weight='bold', pad=10)
        ax.set_xlabel('Pad X (mm)', fontsize=12)
        ax.set_ylabel('Pad Y (mm)', fontsize=12)
        ax.tick_params(labelsize=11)
        ax.grid(alpha=.15)
        ax.plot(0, 0, '+', color='#334155', ms=13, mew=3)
        self.radial, = ax.plot([], [], '--', color=PINK, lw=2)
        self.contact, = ax.plot([], [], 'o', color=PINK, ms=17, mec='white', mew=2, zorder=12)
        self.slider, = ax.plot([], [], 's', color=PURPLE, ms=11, mec='white', mew=1.5, zorder=9)
        self.tip, = ax.plot([], [], '^', color=GREEN, mfc='none', ms=18, mec=GREEN, mew=2.5, zorder=10)
        self.reference, = ax.plot([], [], 'x', color=ORANGE, ms=15, mew=3, zorder=13)
        self.virtual_shaft, = ax.plot([], [], '--', color=BLUE, lw=3, zorder=7)
        self.virtual_pivot, = ax.plot([], [], 'D', color=BLUE, ms=10, mec='white', mew=1.5, zorder=8)
        self.spring, = ax.plot([], [], color=PINK, lw=3, zorder=11)
        self.label = ax.annotate('', xy=(0, 0), xytext=(8, 34), fontsize=13,
                                 color='#9d174d', weight='bold',
                                 arrowprops=dict(arrowstyle='->', color=PINK, lw=2),
                                 bbox=dict(facecolor='white', edgecolor='none', alpha=.9, pad=2), zorder=20)
        self.detail = ax.inset_axes([.65, .04, .32, .32])
        self.detail.set(aspect='equal', xlim=(-6,6), ylim=(-6,6))
        self.detail.set_facecolor('white')
        self.detail.set_title('Palm to virtual tip\nrelative mm',fontsize=10,pad=2)
        self.detail.tick_params(labelsize=8)
        self.detail.grid(alpha=.2)
        self.detail.plot(0, 0, 'o', color=PINK, ms=10, zorder=10)
        self.detail_ref, = self.detail.plot([], [], 'x', color=ORANGE, ms=12, mew=3)
        self.detail_spring, = self.detail.plot([], [], color=PINK, lw=2)
        self.status = ax.text(.02, .02, '', transform=ax.transAxes, fontsize=11,
                              bbox=dict(facecolor='white', edgecolor='none', alpha=.85))

    def update(self, i):
        t = self.t
        to_pad = lambda p: (t.pad_rotation[i].T @ (p - t.pad[i])) * 1000
        a = t.attachment[i] * 1000
        slider, tip, ref = [to_pad(p) for p in (t.alignment_center[i], t.tip[i], t.reference[i])]
        self.contact.set_data([a[0]], [a[1]])
        self.radial.set_data([0,a[0]], [0,a[1]])
        self.tip.set_data([tip[0]], [tip[1]])
        active = t.engaged[i]
        virtual_pivot=to_pad(t.virtual_pivot)
        self.virtual_shaft.set_data([virtual_pivot[0],ref[0]] if active else [],
                                    [virtual_pivot[1],ref[1]] if active else [])
        self.virtual_pivot.set_data([virtual_pivot[0]] if active else [],[virtual_pivot[1]] if active else [])
        for artist, p in ((self.slider, slider), (self.reference, ref)):
            artist.set_data([p[0]] if active else [], [p[1]] if active else [])
        self.label.xy = a[:2]
        self.label.set_text(f'ATTACHMENT\n({a[0]:+.1f}, {a[1]:+.1f}) mm')
        curve = spring_curve(a, ref, width=1.0, turns=5) if active else EMPTY
        self.spring.set_data(curve[:,0], curve[:,1])
        self.detail_ref.set_data([ref[0]-a[0]] if active else [], [ref[1]-a[1]] if active else [])
        self.detail_spring.set_data(curve[:,0]-a[0], curve[:,1]-a[1])
        self.status.set_text(f'Task spring: {np.linalg.norm(ref-a):.1f} mm\n'
                             f'Reference normal: {ref[2]:+.1f} mm' if active else
                             'Task spring inactive\nCaptured attachment retained')


def comparison(trials, output, fps, preview=False, title_label=None):
    count = len(trials)
    if count not in (2,3):
        raise ValueError(f'This layout requires two or three cases; found {count}')
    origins = [t.time[np.flatnonzero(t.engaged)[0]] for t in trials]
    duration = max(t.time[-1] - origin for t, origin in zip(trials, origins))
    center = np.median(np.concatenate([t.pad[t.engaged] for t in trials]), axis=0)
    size = (2560, 1440)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'avc1'), fps, size)
    if not writer.isOpened():
        raise RuntimeError('H.264 encoder unavailable')
    total_frames = 0
    posters = []
    try:
        for chapter in (0, 1):
            fig = plt.figure(figsize=(25.6, 14.4), dpi=100, facecolor='white')
            gs = GridSpec(2, count, figure=fig, left=.035, right=.98, top=.84 if chapter == 0 else .80, bottom=.11,
                          hspace=.32, wspace=.22, height_ratios=(1.12,1.) if chapter == 0 else (2.2,1.))
            title = ('1/2  Pad + task attachment comparison' if chapter == 0 else
                     '2/2  Four alignment springs + joystick response')
            stamp=datetime.datetime.fromisoformat(trials[0].config['created_utc']).astimezone(
                datetime.timezone(datetime.timedelta(hours=9))).strftime('%Y-%m-%d %H:%M JST')
            fig.suptitle(title + ' | '+(title_label or stamp),
                         x=.03, ha='left', y=.985, fontsize=26, weight='bold')
            fig.text(.03, .947, ' / '.join(t.condition.upper() for t in trials)+' | aligned at contact establishment; recorded sequentially | '
                     '1x playback | no robot view', fontsize=17, color='#475569')
            panels, charts, cursors, states = [], [], [], []
            for j, (t, origin) in enumerate(zip(trials, origins)):
                x = (.035 + j * (.945/count))
                fig.text(x, .90, NAMES.get(t.condition, t.condition), fontsize=23, weight='bold', zorder=100)
                state = fig.text(x, .865, '', fontsize=15, color='#334155', zorder=100)
                states.append((state,t,origin))
                if chapter == 0:
                    panels.append((LargePad(fig.add_subplot(gs[0,j], projection='3d', computed_zorder=False),t,center),t,origin))
                    panels.append((PadFace(fig.add_subplot(gs[1,j]),t),t,origin))
                else:
                    scene = SpringScene(fig.add_subplot(gs[0,j], projection='3d'),t)
                    scene.ax.set_title('', loc='left')
                    scene.ax.set_title('Four alignment springs — common slider', fontsize=16, weight='bold', pad=0)
                    scene.ax.set_box_aspect((1,1,1), zoom=.93)
                    scene.ax.tick_params(labelsize=11)
                    scene.ax.texts[-1].set_visible(False)
                    scene.label.set_fontsize(12)
                    for line in scene.springs:
                        line.set_linewidth(4)
                    scene.actual.set_markersize(10)
                    scene.target.set_markersize(10)
                    for label in scene.numbers:
                        label.set_fontsize(13)
                    panels.append((scene,t,origin))
                    ax = fig.add_subplot(gs[1,j])
                    axis = 1 if t.axis == 'y' else 0
                    time = t.time-origin
                    ax.plot(time, np.where(t.engaged,np.rad2deg(t.command[:,axis]),np.nan), '--',
                            color=ORANGE,lw=2.5,label='Command')
                    ax.plot(time,np.where(t.shaft_valid,np.rad2deg(t.shaft[:,axis]),np.nan),
                            color=GREEN,lw=2.5,label='Measured')
                    ax.set(xlim=(0,duration),ylim=(-40,40),xlabel='Seconds since contact establishment',ylabel='Joystick tilt (deg)')
                    ax.legend(fontsize=13,ncol=2)
                    ax.tick_params(labelsize=12)
                    ax.grid(alpha=.2)
                    cursors.append(ax.axvline(0,color='#0f172a',lw=2))
            if chapter == 0:
                fig.text(.035,.052,'PINK: palm attachment + task spring    BLUE dashed: commanded virtual joystick    '
                         'ORANGE X: virtual tip    GREEN: measured joystick    PURPLE: slider',fontsize=16,color='#334155')
            else:
                fig.text(.035,.052,'GOLD COILS: alignment springs S1–S4     PINK SQUARES: slider-frame anchors     '
                         'CYAN CIRCLES: shaft-aligned target anchors',fontsize=16,color='#334155')
            fig.text(.035,.022,'Spatial scales match across cases; markers and coil widths enlarged for visibility. '
                     'Tip uses configured pivot/length. Full target yaw is inferred; pad normals and attachment are logged.',fontsize=13,color='#64748b')
            frames = 1 if preview else int(np.ceil((duration+1.5)*fps))
            for frame in range(frames):
                at = duration*.57 if preview else min(frame/fps,duration)
                for panel,t,origin in panels:
                    i = int(np.clip(np.searchsorted(t.time,at+origin,side='right')-1,0,len(t.rows)-1))
                    panel.update(i)
                for state,t,origin in states:
                    i = int(np.clip(np.searchsorted(t.time,at+origin,side='right')-1,0,len(t.rows)-1))
                    status = 'CONTACT RELEASE' if t.flag('trial_contact_released')[i] else t.modes[i].upper()
                    state.set_text(f't = {at:.2f} s | {status} | EIT contact: {t.rows[i]["sensor_contact"]}')
                for cursor in cursors:
                    cursor.set_xdata([at,at])
                fig.canvas.draw()
                bgr = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()),cv2.COLOR_RGBA2BGR)
                writer.write(bgr)
                total_frames += 1
                if frame == (0 if preview else int(duration*.57*fps)):
                    poster = output.with_name(output.stem+f'_chapter{chapter+1}.png')
                    cv2.imwrite(str(poster),bgr)
                    posters.append(poster.name)
                if frame % (fps*5) == 0:
                    print(f'Chapter {chapter+1}: {frame}/{frames}',flush=True)
            plt.close(fig)
    finally:
        writer.release()
        plt.close('all')
    cap = cv2.VideoCapture(str(output))
    assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == total_frames
    for index in (0,total_frames//2,total_frames-1):
        cap.set(cv2.CAP_PROP_POS_FRAMES,index)
        ok,_ = cap.read()
        assert ok, f'Cannot decode frame {index}'
    cap.release()
    return dict(file=output.name,posters=posters,frames=total_frames,fps=fps,
                duration_s=total_frames/fps,second_chapter_s=total_frames/2/fps,
                contact_alignment_origins_s=origins,source_traces=[str(t.path) for t in trials])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--block',type=Path)
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--fps',type=int,default=12)
    args=parser.parse_args()
    block=args.block or max((p for p in args.root.iterdir() if (p/'block.json').exists()),
                           key=lambda p:json.loads((p/'block.json').read_text()).get('created_utc',''))
    k=yaml.safe_load(Path('/home/birl/franka_ros2_ws/src/franka_description/robots/fr3/kinematics.yaml').read_text())
    trials=[Trial(p,k) for p in sorted(block.glob('repeat_*/*.csv')) if '_ready_' not in p.name]
    trials.sort(key=lambda t:({'baseline':0,'EIT':1,'kinematic':2}.get(t.condition,3),t.ns[0]))
    rotation,source=fit_pad_rotation(trials)
    for trial in trials:
        trial.finish_geometry(rotation,source)
        if not trial.engaged.any():
            raise ValueError(f'No contact establishment in {trial.path}')
        assert np.max(np.linalg.norm(trial.material_attachment[trial.engaged]-trial.point[trial.engaged],axis=1)) < .0001
        assert np.max(trial.virtual_reference_residual_mm)<.001,'Virtual joystick does not match recorded task reference'
    args.output.mkdir(parents=True,exist_ok=True)
    video=comparison(trials,args.output/'latest_all_cases_large.mp4',args.fps,args.preview)
    data=dict(block=str(block),generated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),video=video,
              trials=[t.metrics() for t in trials])
    (args.output/'comparison.json').write_text(json.dumps(data,indent=2)+'\n')
    cells=''.join(f'<tr><td>{html.escape(t.condition)}</td><td>{t.attachment[np.flatnonzero(t.engaged)[0],0]*1000:+.1f}</td>'
                  f'<td>{t.attachment[np.flatnonzero(t.engaged)[0],1]*1000:+.1f}</td></tr>' for t in trials)
    document='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Latest joystick: all cases</title><style>body{font:18px system-ui;max-width:1800px;margin:24px auto;padding:0 20px;color:#0f172a;background:#f1f5f9}video,img{width:100%;background:white}section{background:white;padding:20px;margin:20px 0;border-radius:12px}td,th{padding:8px 24px;text-align:left}a{color:#2563eb}</style>
<h1>Latest block: baseline / EIT / kinematic</h1><p>No robot view. Large pad and joystick views, large attachment markers, face-on pad coordinates and a magnified task-endpoint detail. All cases use the same spatial scale and are aligned at contact establishment.</p>
<video controls preload="metadata" src="latest_all_cases_large.mp4" poster="latest_all_cases_large_chapter1.png"></video>
<p><a href="latest_all_cases_large.mp4">Open MP4 full screen / download</a></p>
<section><p>First chapter: pad, task attachment and face-on coordinates. Second chapter begins at '''+f'{video["second_chapter_s"]:.1f} s'+''': all four alignment springs and commanded/measured joystick tilt.</p>
<p>The pink task attachment is the latched contact point in EIT, pad center in baseline, and projected modeled tip in kinematic. The purple alignment slider is a separate moving point. Pad-face spring detail shows XY projection; the normal offset is printed separately. All conditions were recorded sequentially.</p>
<p><b>Virtual joystick:</b> The blue dashed shaft joins its blue diamond pivot to the orange tip.
The pink task spring connects the selected palm attachment to this virtual tip, whose position is the logged task reference.
Its radius is L minus inward depth (19 mm in this block), and its pivot is fixed by the captured contact attachment.
The green shaft uses the measured angles and configured physical pivot; it is a separate geometry.
The blue virtual tip was checked against every logged task reference during engagement.</p>
<table><tr><th>Case</th><th>Attachment pad X (mm)</th><th>Attachment pad Y (mm)</th></tr>'''+cells+'''</table></section>
<section><h2>Pad and attachment preview</h2><img src="latest_all_cases_large_chapter1.png"></section>
<section><h2>Alignment springs preview</h2><img src="latest_all_cases_large_chapter2.png"></section>
<p><a href="comparison.json">Source traces and diagnostics</a>. Markers and coil widths are enlarged drawing conventions; frame spacing and endpoints preserve physical scale. The joystick tip is a geometry-based estimate.</p></html>'''
    (args.output/'index.html').write_text(document)
    print(args.output/'index.html',flush=True)


if __name__=='__main__':
    main()
