"""Audit first electrical trigger without an oracle contact gate; rerun affected cases."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from concurrent.futures import ProcessPoolExecutor,as_completed
import json,hashlib
import pandas as pd
from run_suite import CACHE,OUT,worker
from experiments import Experiment
def audit(item):
    j,kw,mp=item;key=tuple(sorted(mp.items()))
    if key not in CACHE:CACHE[key]=Experiment(**mp)
    row,_=CACHE[key].run(**kw,event_audit_only=True)
    return j,row
def main():
    primary=json.loads((OUT/'frozen_protocol.json').read_text())['trials'];extra=json.loads((OUT/'exploratory_protocol.json').read_text())['trials']
    unique={};members={}
    for i in primary+extra:
        k=json.dumps({**{x:y for x,y in i[1].items() if x not in ['condition','angle','bias','coarse','stage','target']},'model':i[2]},sort_keys=True)
        if k not in unique:unique[k]=i
        members.setdefault(unique[k][0],[]).append(i)
    audits={};bad=[]
    with ProcessPoolExecutor(max_workers=6) as pool:
        for j,r in pool.map(audit,unique.values()):
            audits[j]=r
            if r['false_latch']:bad.extend(members[j])
    print('Audited',len(unique),'unique acquisition histories; affected',len(bad),flush=True)
    with ProcessPoolExecutor(max_workers=6) as pool:
        corrected=list(pool.map(worker,bad))
    for filename,items in [('trial_metrics.csv',primary),('exploratory_metrics.csv',extra)]:
        ids={i[0] for i in items};p=pd.read_csv(OUT/filename);p['false_latch']=False
        for row in corrected:
            if row['trial_id'] in ids:
                p=p[p.trial_id!=row['trial_id']];p=pd.concat([p,pd.DataFrame([row])],ignore_index=True)
        p.sort_values('trial_id').to_csv(OUT/filename,index=False)
    result={'reason':'Removed ground-truth gating of electrical contact acquisition. The first electrical trigger before actual contact is counted as failure, not silently ignored. Audited all unique initial histories; only affected cases rerun. Other trajectories are identical because they latch on the same valid event.',
            'audits':audits,'corrected_trial_ids':[r['trial_id'] for r in corrected],
            'current_experiments_sha256':hashlib.sha256((OUT.parent/'src/experiments.py').read_bytes()).hexdigest()}
    (OUT/'audit/event_correction.json').write_text(json.dumps(result,indent=2));print('DONE',flush=True)
if __name__=='__main__':main()
