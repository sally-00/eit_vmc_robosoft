#!/usr/bin/env python3
"""Audit matched EIT/baseline records and publish evidence-linked figures/replays."""
import argparse
import csv
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from replay_joystick import Trial, fit_pad_rotation, rows_from
from compare_latest_joystick import comparison


COLORS={'baseline':'#2563eb','eit':'#d97706'}
SELECTED=[('22-42-20','Whole-trial gain; strongest return-phase improvement','negative'),
          ('23-09-47','Large positive-hold gain; full-cycle error slightly worse','positive'),
          ('20-29-09','Positive motion roughly doubles; return remains poor','positive')]


def read_trace(path, config):
    rows=rows_from(path)
    ns=np.array([int(r['host_time_ns']) for r in rows],dtype=np.int64)
    time=(ns-ns[0])/1e9
    active=np.array([r['mode'] in ('preload','interact') for r in rows])
    if not active.any():
        return None
    time-=time[np.flatnonzero(active)[0]]
    interact=np.array([r['mode']=='interact' for r in rows])
    fresh=np.array([r.get('joystick_shaft_fresh')=='True' for r in rows])
    valid=interact&fresh
    if not valid.any():
        return None
    col=lambda key:np.array([float(r[key]) for r in rows])
    vec=lambda prefix:np.column_stack([col(prefix+'_'+a) for a in 'xyz'])
    axis=rows[-1].get('joystick_axis') or config.get('axis') or 'x'
    sel=1 if axis=='y' else 0
    cmd=np.rad2deg(np.column_stack((col('theta_d_rad'),col('theta_d_y_rad'))))
    measured=np.rad2deg(np.column_stack((col('joystick_shaft_angle_rad'),col('joystick_shaft_angle_y_rad'))))
    shaft=np.c_[np.tan(np.deg2rad(measured)),np.ones(len(rows))]
    shaft/=np.linalg.norm(shaft,axis=1)[:,None]
    alignment=np.rad2deg(np.arccos(np.clip(-np.sum(vec('pad_normal_base')*shaft,axis=1),-1,1)))
    return dict(rows=rows,time=time,active=active,valid=valid,axis=axis,sel=sel,cmd=cmd[:,sel],
                measured=measured[:,sel],alignment=alignment,
                point_error_mm=np.linalg.norm(vec('reference_m')-vec('point_m'),axis=1)*1000,
                force_N=np.linalg.norm(vec('requested_force_N'),axis=1),
                contact=np.array([r['sensor_contact']=='True' for r in rows]),
                clipped=np.array([r.get('joystick_shaft_clipped')=='True' for r in rows]))


def common_samples(data):
    start=max(d['time'][d['valid']][0] for d in data.values())
    end=min(d['time'][d['valid']][-1] for d in data.values())
    grid=np.arange(start,end+1e-8,.02)
    keep=np.ones(len(grid),bool)
    for d in data.values():
        left=np.clip(np.searchsorted(d['time'],grid,side='right')-1,0,len(d['time'])-1)
        right=np.minimum(left+1,len(d['time'])-1)
        keep &= d['valid'][left]&d['valid'][right]&((d['time'][right]-d['time'][left])<=.1)
    grid=grid[keep]
    sampled={c:{key:np.interp(grid,d['time'],d[key]) for key in
                ('cmd','measured','alignment','point_error_mm','force_N')} for c,d in data.items()}
    return grid,sampled


def audit(root):
    blocks=sorted((p for p in root.iterdir() if (p/'block.json').exists()),
                  key=lambda p:json.loads((p/'block.json').read_text()).get('created_utc',''))
    pairs=[]
    for block in blocks:
        config=json.loads((block/'block.json').read_text())
        for repeat in sorted(block.glob('repeat_*')):
            paths={c:sorted(repeat.glob('joystick_'+c+'_*.csv')) for c in ('baseline','eit')}
            if not all(paths.values()):
                continue
            paths={c:ps[-1] for c,ps in paths.items()}
            data={c:read_trace(p,config) for c,p in paths.items()}
            if any(d is None for d in data.values()):
                continue
            grid,sampled=common_samples(data)
            if len(grid)<10:
                continue
            amplitude=float(config['checked_controller_parameters'].get('joystick_angle_deg',20.))
            phase_cmd=np.mean([s['cmd'] for s in sampled.values()],axis=0)
            conditions={}
            for c,d in data.items():
                s=sampled[c];error=s['measured']-s['cmd'];rows=d['rows']
                rmse=lambda mask:float(np.sqrt(np.mean(error[mask]**2))) if mask.any() else None
                conditions[c]=dict(source=str(paths[c]),axis=d['axis'],final_mode=rows[-1]['mode'],
                    faults=sorted(set(r['fault'] for r in rows)-{'none'}),
                    contact_released=rows[-1].get('trial_contact_released')=='True',
                    common_rmse_deg=rmse(np.ones(len(grid),bool)),
                    positive_hold_rmse_deg=rmse(phase_cmd>.9*amplitude),
                    negative_hold_rmse_deg=rmse(phase_cmd<-.9*amplitude),
                    positive_peak_deg=float(np.max(d['measured'][d['valid']])),
                    clipped_fraction=float(np.mean(d['clipped'][d['valid']])),
                    contact_false_fraction=float(np.mean(~d['contact'][d['valid']])),
                    alignment_rmse_deg=float(np.sqrt(np.mean(s['alignment']**2))),
                    task_point_rmse_mm=float(np.sqrt(np.mean(s['point_error_mm']**2))))
            improvement=100*(1-conditions['eit']['common_rmse_deg']/conditions['baseline']['common_rmse_deg'])
            complete=all(m['final_mode']=='done' and not m['faults'] and not m['contact_released'] for m in conditions.values())
            phases={}
            for phase in ('positive','negative'):
                key=phase+'_hold_rmse_deg';a=conditions['baseline'][key];b=conditions['eit'][key]
                phases[phase]=100*(1-b/a) if a and b is not None else None
            pairs.append(dict(block=block.name,block_path=str(block),repeat=repeat.name,
                complete_pair=complete,improvement_pct=improvement,phase_improvement_pct=phases,
                common_start_s=float(grid[0]),common_end_s=float(grid[-1]),common_samples=len(grid),
                command_difference_max_deg=float(np.max(np.abs(sampled['eit']['cmd']-sampled['baseline']['cmd']))),
                command_amplitude_deg=amplitude,conditions=conditions))
    pairs.sort(key=lambda p:p['improvement_pct'],reverse=True)
    return blocks,pairs


def load_geometry(pair):
    k=yaml.safe_load(Path('/home/birl/franka_ros2_ws/src/franka_description/robots/fr3/kinematics.yaml').read_text())
    trials=[Trial(Path(pair['conditions'][c]['source']),k) for c in ('baseline','eit')]
    rotation,source=fit_pad_rotation(trials)
    for trial in trials:
        trial.finish_geometry(rotation,source)
        assert np.max(trial.virtual_reference_residual_mm)<.001
        assert np.max(np.linalg.norm(trial.material_attachment[trial.engaged]-trial.point[trial.engaged],axis=1))*1000<.01
    return trials


def figures(pair,folder,title,phase):
    folder.mkdir(parents=True,exist_ok=True)
    config=json.loads((Path(pair['block_path'])/'block.json').read_text())
    data={c:read_trace(Path(m['source']),config) for c,m in pair['conditions'].items()}
    grid,sampled=common_samples(data)
    phase_cmd=np.mean([s['cmd'] for s in sampled.values()],axis=0)
    focus=phase_cmd>(.9*pair['command_amplitude_deg']) if phase=='positive' else phase_cmd<(-.9*pair['command_amplitude_deg'])
    fig,axes=plt.subplots(3,2,figsize=(16,12),sharex=True)
    axes=axes.ravel()
    fig.suptitle(pair['block'].split('_2026')[0]+' | '+pair['block'].split('2026-10-25_')[-1][:8].replace('-',':')+' JST\n'+title,fontsize=17,weight='bold')
    for c,d in data.items():
        color=COLORS[c];name='Without EIT' if c=='baseline' else 'With EIT'
        axes[0].plot(d['time'],np.where(d['active'],d['cmd'],np.nan),'--',color=color,lw=1,alpha=.55)
        axes[0].plot(d['time'],d['measured'],color=color,lw=1.5,label=name)
        s=sampled[c]
        axes[1].plot(grid,s['measured']-s['cmd'],color=color,label=name)
        axes[2].plot(d['time'],d['alignment'],color=color,lw=1,label=name)
        axes[3].plot(d['time'],d['point_error_mm'],color=color,lw=1,label=name)
        axes[4].plot(d['time'],d['force_N'],color=color,lw=1,label=name)
        offset=0 if c=='baseline' else 1.4
        axes[5].step(d['time'],offset+d['contact'],where='post',color=color,lw=1.3,label=name+' contact')
        clipped=d['time'][d['clipped']&d['valid']]
        axes[0].scatter(clipped,np.interp(clipped,d['time'],d['measured']),s=7,color=color,marker='x')
        m=pair['conditions'][c]
        axes[0].text(.01,.07 if c=='baseline' else .01,f'{name}: shared-window RMSE {m["common_rmse_deg"]:.2f}°',
                     transform=axes[0].transAxes,color=color,fontsize=10)
    labels=['Joystick tilt (deg); dashed = command','Measured − commanded (deg)',
            'Pad / measured-shaft angle (deg)','Task point to virtual tip (mm)',
            'Requested force norm (N)','EIT contact flag (rows offset)']
    for ax,label in zip(axes,labels):
        ax.set_ylabel(label);ax.grid(alpha=.2)
        ax.set_xlim(0,max(d['time'][-1] for d in data.values()))
        if focus.any():
            lo,hi=grid[focus][0],grid[focus][-1]
            ax.axvspan(lo,hi,color='#16a34a',alpha=.09)
        ax.axvline(pair['common_start_s'],color='#64748b',ls=':',lw=.7)
        ax.axvline(pair['common_end_s'],color='#64748b',ls=':',lw=.7)
    axes[0].legend(loc='upper left',ncol=2,fontsize=10)
    axes[1].axhline(0,color='#64748b',lw=.7)
    axes[5].set_yticks([0,1,1.4,2.4],['baseline off','baseline on','EIT off','EIT on'])
    for ax in axes[-2:]:ax.set_xlabel('Seconds since contact establishment')
    fig.text(.04,.015,'Green shading: highlighted command-hold phase. Dotted boundaries: shared valid interaction window. '
             '× marks bounded / clipped ADC estimates. Contact is electrical classification; force is a controller request.',fontsize=10,color='#475569')
    fig.tight_layout(rect=(0,.035,1,.93))
    fig.savefig(folder/'tracking_and_diagnostics.png',dpi=140)
    plt.close(fig)
    trials=load_geometry(pair)
    fig,axes=plt.subplots(1,2,figsize=(14,6))
    geometry=[]
    for ax,t in zip(axes,trials):
        i=np.flatnonzero(t.engaged)[0];convert=lambda p:(np.asarray(p)-t.pivot)*1000
        pad=t.pad[i];angle=np.linspace(0,2*np.pi,120)
        rim=pad+np.column_stack((.045*np.cos(angle),.045*np.sin(angle),np.zeros_like(angle)))@t.pad_rotation[i].T
        rim=convert(rim);ax.fill(rim[:,0],rim[:,1],color='#e2e8f0',alpha=.6)
        ax.plot(rim[:,0],rim[:,1],color='#64748b',lw=1)
        pc,attach,virtual=convert(pad),convert(t.material_attachment[i]),convert(t.virtual_pivot)
        ax.plot([pc[0],attach[0]],[pc[1],attach[1]],'--',color='#db2777',lw=2)
        ax.scatter(*pc[:2],marker='+',s=100,color='#334155',label='Pad center')
        ax.scatter(*attach[:2],s=95,color='#db2777',label='Task attachment')
        ax.scatter(*virtual[:2],marker='D',s=75,color='#2563eb',label='Virtual pivot (XY projection)')
        ax.scatter(0,0,s=90,marker='x',color='#15803d',label='Configured physical pivot')
        measured=convert(t.tip[t.engaged]);target=convert(t.virtual_tip[t.engaged])
        ax.plot(measured[:,0],measured[:,1],color='#15803d',lw=1.4,alpha=.8,label='Measured-tip model path')
        ax.plot(target[:,0],target[:,1],'--',color='#2563eb',lw=1.4,label='Virtual-tip path')
        mismatch=float(np.linalg.norm(virtual[:2]));geometry.append(dict(condition=t.condition,
            virtual_pivot_horizontal_offset_mm=mismatch,attachment_pad_mm=(t.attachment[i]*1000).tolist(),
            virtual_pivot_base_m=t.virtual_pivot.tolist(),virtual_radius_mm=t.virtual_radius*1000,
            virtual_tip_reference_error_mm=float(np.max(t.virtual_reference_residual_mm))))
        ax.set_title(t.condition+f' | virtual/physical pivot XY offset {mismatch:.1f} mm',fontsize=13,weight='bold')
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('Base X from configured pivot (mm)');ax.set_ylabel('Base Y from configured pivot (mm)')
    points=np.concatenate([ax.dataLim.get_points() for ax in axes]);lo=np.min(points,axis=0)-5;hi=np.max(points,axis=0)+5
    for ax in axes:ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1])
    axes[0].legend(fontsize=8,loc='best')
    fig.suptitle('Captured geometry and model paths — same scale',fontsize=17,weight='bold')
    fig.text(.04,.015,'A smaller model discrepancy is consistent with a better task geometry; it does not independently measure physical contact or prove causation.',fontsize=10,color='#475569')
    fig.tight_layout(rect=(0,.045,1,.93));fig.savefig(folder/'contact_geometry.png',dpi=140);plt.close(fig)
    return geometry


def overview(pairs,output):
    fig,ax=plt.subplots(figsize=(11,11))
    labels=[]
    for i,p in enumerate(pairs):
        a=p['conditions']['baseline']['common_rmse_deg'];b=p['conditions']['eit']['common_rmse_deg']
        ax.plot([a,b],[i,i],color='#cbd5e1',lw=2)
        ax.scatter(a,i,color=COLORS['baseline'],s=35,label='Without EIT' if i==0 else None)
        ax.scatter(b,i,color=COLORS['eit'],s=35,label='With EIT' if i==0 else None)
        labels.append(p['block'].split('_2026')[0]+' '+p['block'].split('2026-10-25_')[-1][:8].replace('-',':')+
                      ('' if p['complete_pair'] else ' [release / fault / incomplete]'))
    ax.set_yticks(range(len(pairs)),labels);ax.invert_yaxis();ax.set_xlabel('Angle RMSE on shared valid interaction window (deg)')
    ax.grid(axis='x',alpha=.2);ax.legend(loc='lower right')
    ax.set_title(f'All {len(pairs)} matched pairs — ranked by percentage RMSE reduction',fontsize=15,weight='bold')
    fig.tight_layout();fig.savefig(output/'all_pairs_overview.png',dpi=140);plt.close(fig)


def report(output):
    audit_data=json.loads((output/'audit.json').read_text());selected=audit_data['selected'];pairs=audit_data['pairs']
    cards=[]
    for item in selected:
        folder=item['folder'];p=item['pair'];a=p['conditions']['baseline'];b=p['conditions']['eit']
        phase=item['phase'];key=phase+'_hold_rmse_deg'
        video_path=output/folder/'video.json'
        movie=''
        if video_path.exists():
            movie=f'<video controls preload="metadata" src="{folder}/comparison.mp4" poster="{folder}/comparison_chapter1.png"></video>'
        geometry=item['geometry'];offset=' → '.join(f'{g["virtual_pivot_horizontal_offset_mm"]:.1f}' for g in geometry)
        interval=item.get('focus_interval_s')
        focus_text=(f'Focus on {interval[0]:.1f}–{interval[1]:.1f} seconds in the first video chapter. '
                    if interval else '')
        cards.append(f'<section><h2>{html.escape(item["title"])}</h2><p>{html.escape(p["block"])}</p>'
            f'<p><b>Shared-window angle RMSE:</b> {a["common_rmse_deg"]:.2f}° → {b["common_rmse_deg"]:.2f}° '
            f'({p["improvement_pct"]:+.1f}% improvement). <b>{phase.capitalize()} hold:</b> {a[key]:.2f}° → {b[key]:.2f}° '
            f'({p["phase_improvement_pct"][phase]:+.1f}% improvement).</p>'
            f'<p>Positive measured peak: {a["positive_peak_deg"]:.1f}° → {b["positive_peak_deg"]:.1f}°. '
            f'Clipped interaction samples: {a["clipped_fraction"]*100:.1f}% → {b["clipped_fraction"]*100:.1f}%. '
            f'Configured-physical versus virtual pivot XY offset: {offset} mm.</p>'
            f'<p>{focus_text}The contact attachment shifts the task geometry toward the configured joystick pivot. This is a plausible mechanism; '
            f'the gain is phase-specific when the whole-trial percentage is negative. The paired conditions were sequential, with baseline first.</p>'
            f'{movie}<p><a href="{folder}/comparison.mp4">Open comparison MP4 full screen</a> · '
            f'<a href="{folder}/tracking_and_diagnostics.png">Full-size tracking figure</a> · '
            f'<a href="{folder}/contact_geometry.png">Full-size geometry figure</a></p>'
            f'<img src="{folder}/tracking_and_diagnostics.png"><img src="{folder}/contact_geometry.png"></section>')
    doc='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>EIT improvements in previous joystick runs</title><style>body{font:18px system-ui;max-width:1700px;margin:28px auto;padding:0 24px;background:#f1f5f9;color:#0f172a;line-height:1.55}section{background:white;padding:24px;margin:24px 0;border-radius:12px}video,img{width:100%}a{color:#2563eb}h2{font-size:23px}</style>
<h1>Previous joystick runs: where EIT helps</h1><p>Comparison of '''+str(audit_data['blocks'])+' blocks and '+str(len(pairs))+''' matched baseline/EIT pairs. Both conditions must have fresh shaft samples in Interact. Metrics use the same time window at 20 ms spacing; interpolation across gaps longer than 100 ms is excluded. Time zero is contact establishment. Positive/negative holds use the same ±90% command-amplitude threshold for both cases.</p>
<section><h2>What the full records support</h2><p>The largest whole-trial RMSE reduction among completed pairs is about 15%, at 22:42. Larger improvements appear during specific phases, including the 23:09 positive hold and positive motion in the 20:29 folder marked _good. These phase improvements do not imply lower full-cycle error. ADC endpoint estimates remain bounded at ±35° and are flagged.</p>
<p>The 23:32 EIT trace has a low unpaired RMSE because it ends after contact release; it is not a successful full-cycle comparison. All paired records, including incomplete attempts, remain in the overview and audit.</p>
<p>Video colors: green measured joystick; blue dashed virtual joystick; pink palm attachment/task spring; orange virtual tip; purple alignment slider. No robot view. Both chapters preserve the large geometry layout.</p></section>'''+''.join(cards)+'''<section><h2>All matched pairs</h2><img src="all_pairs_overview.png"><p><a href="audit.json">Full audit, exact source paths and metrics</a></p></section>
<p>The physical pivot, length and tip path are configured geometry, not independent contact measurements. Gains can differ between blocks; no pooled causal or significance claim is made. Within each selected pair, the recorded block parameters are shared. Electrical contact flags and requested forces do not independently establish object success.</p></html>'''
    (output/'index.html').write_text(doc)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--render',type=int,choices=(1,2,3))
    parser.add_argument('--report-only',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if args.report_only:
        report(args.output);return
    if args.render:
        item=json.loads((args.output/'audit.json').read_text())['selected'][args.render-1]
        trials=load_geometry(item['pair']);folder=args.output/item['folder']
        metadata=comparison(trials,folder/'comparison.mp4',12,title_label='2026-10-25 '+item['pair']['block'].split('2026-10-25_')[-1][:8].replace('-',':')+' JST')
        (folder/'video.json').write_text(json.dumps(metadata,indent=2)+'\n');return
    blocks,pairs=audit(args.root);selected=[]
    for index,(stamp,title,phase) in enumerate(SELECTED,1):
        pair=next(p for p in pairs if stamp in p['block']);folder=f'case_{index}'
        geometry=figures(pair,args.output/folder,title,phase)
        selected.append(dict(folder=folder,title=title,phase=phase,pair=pair,geometry=geometry))
    overview(pairs,args.output)
    (args.output/'audit.json').write_text(json.dumps(dict(blocks=len(blocks),pairs=pairs,selected=selected),indent=2)+'\n')
    report(args.output)
    for item in selected:
        p=item['pair'];print(p['block'],'whole-trial improvement',round(p['improvement_pct'],1),'phase improvement',p['phase_improvement_pct'])


if __name__=='__main__':
    main()
