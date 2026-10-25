"""Freeze final trial schedule before evaluation; parallelize independent episodes."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import json,time,hashlib,argparse
import numpy as np
import pandas as pd
from concurrent.futures import ProcessPoolExecutor,as_completed
from experiments import Experiment,s0,OUT
ROOT=OUT.parent
CACHE={}
def worker(item):
    j,kw,modelpars=item
    key=tuple(sorted(modelpars.items()))
    if key not in CACHE:CACHE[key]=Experiment(**modelpars)
    row,tr=CACHE[key].run(**kw)
    row.update(modelpars);row['trial_id']=j
    np.savez_compressed(OUT/'trial_data'/f'trial_{j:04d}.npz',trace=tr,row=json.dumps(row))
    return row

def schedule():
    items=[]
    def add(k,m=None):items.append((len(items),k,{} if m is None else m))
    for x in [-.03,0,.03]:
        for c in ['fixed','eit','oracle']:
            for seed in range(700,705):add(dict(stage='S1',condition=c,offset=x,angle=0,seed=seed),dict(spring=1000.))
    for x in [-.03,-.015,0,.015,.03]:
        for c in ['fixed','eit','oracle']:
            for a in [-5,5]:
                for seed in range(710,715):add(dict(stage='S2',condition=c,offset=x,angle=a,seed=seed))
    for b in [-.02,-.01,-.005,-.002,0,.002,.005,.01,.02]:
        for a in [-5,5]:
            for seed in range(720,725):add(dict(stage='S3',condition='oracle',offset=.027,angle=a,seed=seed,bias=b))
    for coarse in [.005,.01,.02]:
        for seed in range(720,725):add(dict(stage='S3',condition='eit',offset=.027,angle=5,seed=seed,coarse=coarse))
    for delay in [0,.02,.05,.1]:
        for seed in range(720,725):add(dict(stage='S3',condition='eit',offset=.027,angle=5,seed=seed,delay=delay))
    for rate in [1.,5.,10.]:
        for seed in range(720,725):add(dict(stage='S3',condition='eit',offset=.027,angle=5,seed=seed,rate=rate))
    for noise in [.001,.003,.01]:
        for seed in range(720,725):add(dict(stage='S3',condition='eit',offset=.027,angle=5,seed=seed,noise=noise))
    for mu in [.2,.8]:
        for soft in [.004,.016]:
            for c in ['fixed','eit','oracle']:
                for seed in range(730,733):add(dict(stage='S3',condition=c,offset=.03,angle=5,seed=seed),dict(mu=mu,soft=soft))
    for seed in range(735,740):add(dict(stage='S3',condition='eit',offset=.027,angle=5,seed=seed,shift=True))
    for spring in [500.,1500.]:
        for target in [2.,4.,6.]:
            for c in ['nominal','force_eit','force_oracle']:
                for seed in range(740,745):add(dict(stage='S4',condition=c,offset=0.,angle=0,seed=seed,target=target),dict(spring=spring))
    # Numerical convergence, separate from the primary experiment tables.
    for c in ['fixed','eit','oracle']:
        add(dict(stage='S2_check',condition=c,offset=.03,angle=5,seed=750),dict(dt=.0005))
    return items[:-3] # dedicated step refinement is run separately with the S2 protocol

def main():
    p=argparse.ArgumentParser();p.add_argument('--workers',type=int,default=4);args=p.parse_args()
    OUT.mkdir(exist_ok=True);(OUT/'trial_data').mkdir(exist_ok=True);s0()
    items=schedule();schedulepath=OUT/'frozen_protocol.json'
    protocol={'success':{'rotation_error_deg':1,'contact_fraction':.99,'peak_force_N':10,'anchor_drift_mm':2},'note':'Pilot seed 100 excluded from final trials. Settings frozen before this schedule executes.',
              'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src').glob('*.py')},'trials':items}
    if not schedulepath.exists():schedulepath.write_text(json.dumps(protocol,indent=2))
    done=[];pending=[]
    for item in items:
        path=OUT/'trial_data'/f'trial_{item[0]:04d}.npz'
        if path.exists():done.append(json.loads(str(np.load(path)['row'])))
        else:pending.append(item)
    print('Trials',len(items),'resume',len(done),'pending',len(pending),flush=True);t=time.time()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(worker,item):item[0] for item in pending}
        for f in as_completed(futures):
            done.append(f.result())
            if len(done)%10==0:
                pd.DataFrame(done).sort_values('trial_id').to_csv(OUT/'trial_metrics.csv',index=False)
                print('Completed',len(done),'/',len(items),'elapsed',round(time.time()-t,1),'s',flush=True)
    pd.DataFrame(done).sort_values('trial_id').to_csv(OUT/'trial_metrics.csv',index=False)
    print('COMPLETE',time.time()-t,flush=True)
if __name__=='__main__':main()
