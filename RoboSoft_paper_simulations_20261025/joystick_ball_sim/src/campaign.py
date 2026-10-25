"""Frozen, resumable paired evaluation. Development seeds are excluded."""
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import argparse
import hashlib
import json
import os
import platform
import time
import numpy as np
import mujoco
from scene import ROOT, BASE
from trial import run_trial

OUT = ROOT/'results'


def schedule():
    rows = []
    policies = [('timed','centre'), ('timed','eit'), ('event','centre'),
                ('event','eit'), ('event','oracle')]
    def add(group, **kwargs):
        cfg = dict(task='joystick', switching='event', attachment='eit',
                   mechanism='mechanism', gap=.006, offset=0., seed=300,
                   nominal_time=1.0, mu=.7, recess=.003, dt=.001,
                   distance=.025, horizon=16., group=group)
        cfg.update(kwargs)
        cfg['trial_id'] = len(rows)
        rows.append(cfg)
    for task, offsets, seeds in [('joystick',[-.025,0,.025],range(300,303)),
                                  ('ball',[-.015,0,.015],range(400,403))]:
        for offset in offsets:
            for gap in [.002,.006,.014]:
                for seed in seeds:
                    for switching, attachment in policies:
                        add('J1' if task=='joystick' else 'B1', task=task,
                            offset=offset, gap=gap, seed=seed,
                            switching=switching, attachment=attachment,
                            horizon=16. if task=='joystick' else 28.)
                    add('J2' if task=='joystick' else 'B2', task=task,
                        offset=offset, gap=gap, seed=seed, mechanism='point',
                        horizon=16. if task=='joystick' else 28.)
    for k in [.05,.4]:
        for offset in [-.025,0.,.025]:
            for seed in range(500,503):
                for mech in ['mechanism','point']:
                    add('J3',stick_k=k,offset=offset,seed=seed,mechanism=mech)
    for offset in [-.01,.01]:
        for seed in range(600,603):
            for switching,attachment in policies:
                add('B3',task='ball',offset=offset,seed=seed,switching=switching,
                    attachment=attachment,distance=.08,horizon=40.)
            add('B3',task='ball',offset=offset,seed=seed,mechanism='point',
                distance=.08,horizon=40.)
    for offset in [-.01,.01]:
        for seed in range(700,703):
            for switching in ['timed','event']:
                for attachment in ['centre','eit']:
                    add('B4',task='ball',offset=offset,seed=seed,
                        switching=switching,attachment=attachment,
                        distance=.06,horizon=35.,interrupt=True,interrupt_at=2.5)
            add('B4',task='ball',offset=offset,seed=seed,mechanism='point',
                distance=.06,horizon=35.,interrupt=True,interrupt_at=2.5)
    for radius in [.02,.025,.03]:
        for mu in [.1,.4,.8]:
            for seed in range(800,803):
                for attachment in ['centre','eit']:
                    add('B5',task='ball',radius=radius,mu=mu,seed=seed,
                        attachment=attachment,distance=.04,horizon=35.)
    for recess in [0.,.003,.006]:
        for seed in range(900,903):
            for attachment in ['centre','eit']:
                add('B6',task='ball',recess=recess,offset=.020,seed=seed,
                    attachment=attachment,distance=.035,horizon=30.)
    for task,offset in [('joystick',.025),('ball',.015)]:
        for seed in range(1000,1003):
            for rate in [5.,10.,20.]:
                add('S1',task=task,offset=offset,seed=seed,rate=rate,
                    horizon=16. if task=='joystick' else 28.)
            add('S1',task=task,offset=offset,seed=seed,sensor_shift=True,
                horizon=16. if task=='joystick' else 28.)
        for seed in range(1100,1103):
            for mech in ['mechanism','point']:
                add('D1',task=task,seed=seed,mechanism=mech,resistance=True,
                    distance=.04,horizon=16. if task=='joystick' else 30.)
        for seed in range(1200,1203):
            add('S2',task=task,offset=offset,seed=seed,noise=.01,gap=.014,
                horizon=16. if task=='joystick' else 28.)
    return rows


def hashes():
    files = list((ROOT/'src').glob('*.py'))
    files += [BASE/'src/robot.py', ROOT.parent/'RoboSoft_EIT_VMC_Draft/manuscript.tex',
              ROOT/'results/eit_estimator_restored.joblib']
    return {str(p.relative_to(ROOT.parent)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in files}


def freeze():
    OUT.mkdir(exist_ok=True)
    rows = schedule()
    record = dict(trials=rows, count=len(rows), source_sha256=hashes(),
                  max_physical_steps=sum(round(c['horizon']/c['dt']) for c in rows),
                  development_seeds=[100,101,102],
                  success={'joystick_mean_error_deg':2.,'joystick_contact_fraction':.95,
                           'ball_endpoint_mm':5.,'ball_cross_path_mm':5.,
                           'peak_pad_force_N':12.,'peak_rim_force_N':.2,
                           'premature_engagements':0},
                  common_guards={'pad_force_N':15.,'rim_force_N':1.,'joint_speed_rad_s':5.},
                  note='Frozen before evaluation. Exact FEM voltages and restored original estimator. '
                       'All ball controllers receive the same tactile progress estimator. '
                       'No EIT force output enters control. Development is excluded.',
                  environment={'python':platform.python_version(),'mujoco':mujoco.__version__,
                               'numpy':np.__version__})
    (OUT/'frozen_protocol.json').write_text(json.dumps(record,indent=2))
    print(json.dumps({k:record[k] for k in ['count','max_physical_steps','environment']},indent=2))


def worker(cfg):
    return run_trial(cfg, OUT/'trials'/f"trial_{cfg['trial_id']:04d}.npz")


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--freeze',action='store_true')
    parser.add_argument('--workers',type=int,default=4); args=parser.parse_args()
    if args.freeze:
        if (OUT/'trials').exists() and any((OUT/'trials').glob('*.npz')):
            raise RuntimeError('Evaluation traces exist; preserve this campaign instead of re-freezing it.')
        freeze(); return
    protocol=json.loads((OUT/'frozen_protocol.json').read_text())
    current=hashes()
    for path,sha in protocol['source_sha256'].items():
        if path in current and current[path]!=sha:
            raise RuntimeError(f'Frozen source changed: {path}')
    rows=protocol['trials']; completed=[]; pending=[]
    for cfg in rows:
        path=OUT/'trials'/f"trial_{cfg['trial_id']:04d}.npz"
        if path.exists():
            with np.load(path) as saved: completed.append(json.loads(str(saved['metrics'])))
        else: pending.append(cfg)
    start=time.time()
    def status():
        record=dict(state='running' if len(completed)<len(rows) else 'complete',
                    completed=len(completed),total=len(rows),
                    physical_steps=sum(r['physical_steps'] for r in completed),
                    max_physical_steps=protocol['max_physical_steps'],
                    elapsed_s=time.time()-start,workers=args.workers,
                    successes=sum(r['success'] for r in completed))
        tmp=OUT/'STATUS.tmp';tmp.write_text(json.dumps(record,indent=2));tmp.replace(OUT/'STATUS.json')
    status()
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures={executor.submit(worker,cfg):cfg['trial_id'] for cfg in pending}
        for future in as_completed(futures):
            row=future.result();completed.append(row);status()
            print(json.dumps({k:row.get(k) for k in ['trial_id','group','task','switching',
                  'attachment','mechanism','success','target_error','peak_force_N','guard']}),flush=True)
    (OUT/'metrics.json').write_text(json.dumps(sorted(completed,key=lambda r:r['trial_id']),indent=2))


if __name__=='__main__':main()
