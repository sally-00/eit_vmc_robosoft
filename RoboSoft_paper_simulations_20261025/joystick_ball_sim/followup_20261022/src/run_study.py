"""Isolated exploratory/frozen follow-up runner. All attempts retained."""
import argparse,json,hashlib,time,sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import numpy as np
from trial_followup import run_trial
ROOT=Path(__file__).resolve().parents[1]
PROJECT=ROOT.parent

def base(**kw):
    c=dict(task='ball',switching='event',attachment='centre',mechanism='mechanism',
        progress_source='eit',seed=1500,gap=.006,offset=0.,mu=.7,recess=.003,dt=.001,
        distance=.020,horizon=14.,fixed_duration=8.,allow_regrasp=False,
        orientation_k=3.,score_error_mm=5.,settle_after=3.3)
    c.update(kw);return c

def pilot():
    rows=[]
    def add(group,**kw):rows.append(base(group=group,**kw))
    for case,extra in [('nominal',{}),('push_plus',{'disturbances':[dict(start=2.,duration=.25,force=4.)]}),
                       ('push_minus',{'disturbances':[dict(start=2.,duration=.25,force=-4.)]}),
                       ('table_plus',{'table_shift':.020}),('low_friction',{'mu':.08})]:
        for source in ['odometry','eit','oracle']:
            add('feedback',case=case,progress_source=source,**extra)
    for k in [.3,1.,3.]:
        for offset in [-.025,.025]:
            for attachment in ['centre','latched','eit','oracle']:
                add('attachment',case=f'k{k}_offset{offset}',orientation_k=k,offset=offset,attachment=attachment)
    for case,extra in [('nominal',{}),('resistance',{'resistance':True}),
                       ('stop_restart',{'goal_schedule':[dict(start=0.,distance=.010),dict(start=3.5,distance=.020)]})]:
        for mech,scale in [('point',1.),('mechanism',1.),('mechanism',0.)]:
            add('dynamics',case=case,mechanism=mech,reaction_scale=scale,**extra)
    return rows

def hashes():
    files=list((ROOT/'src').glob('*.py'))+[PROJECT/'src/scene.py',PROJECT/'src/sensing.py',
          PROJECT.parent/'ur3e_sim/src/robot.py',PROJECT/'src/electrical.py',
          PROJECT/'results/eit_estimator_restored.joblib']
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}

def worker(item):
    cfg,path=item
    return run_trial(cfg,Path(path))

def execute(rows,name,workers=4):
    out=ROOT/name;out.mkdir(exist_ok=True);(out/'trials').mkdir(exist_ok=True)
    for i,c in enumerate(rows):c['trial_id']=i
    prot=out/'protocol.json'
    record=dict(name=name,trials=rows,source_sha256=hashes(),created_unix=time.time(),
        note='Exploratory development' if name.startswith('pilot') else 'Schedule frozen before this evaluation; selected after pilot inspection.',
        scoring='At 8 s after first engagement, last-second mean absolute true goal error <=5 mm, mean speed <=4 mm/s, crosspath <=5 mm, peak pad <=12 N, rim <0.2 N, no guard/premature. All attempts counted.',
        physics_step_budget=sum(round(c['horizon']/c['dt']) for c in rows))
    if prot.exists():
        old=json.loads(prot.read_text());assert old['trials']==rows
        for f,h in old['source_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h,f
    else:prot.write_text(json.dumps(record,indent=2))
    pending=[];results=[];started=time.time()
    for c in rows:
        p=out/'trials'/f"trial_{c['trial_id']:04d}.npz"
        if p.exists():
            with np.load(p) as z:results.append(json.loads(str(z['metrics'])))
        else:pending.append((c,str(p)))
    def status():
        (out/'STATUS.json').write_text(json.dumps(dict(state='complete' if len(results)==len(rows) else 'running',
            completed=len(results),total=len(rows),elapsed_s=time.time()-started,
            steps=sum(r['physical_steps'] for r in results)),indent=2))
    status()
    with ProcessPoolExecutor(max_workers=workers) as pool:
        fs=[pool.submit(worker,x) for x in pending]
        for f in as_completed(fs):
            row=f.result();results.append(row);status()
            print(json.dumps({k:row.get(k) for k in ['trial_id','group','case','progress_source','attachment','mechanism','reaction_scale','common_success','late_error_mm','peak_force_N','peak_pitch_deg','progress_mae_mm','guard']}),flush=True)
    results.sort(key=lambda r:r['trial_id']);(out/'metrics.json').write_text(json.dumps(results,indent=2))
    return results

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',default='pilot');p.add_argument('--schedule');p.add_argument('--workers',type=int,default=4);a=p.parse_args()
    rows=json.loads(Path(a.schedule).read_text()) if a.schedule else pilot()
    execute(rows,a.name,a.workers)
