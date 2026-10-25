"""Exploratory follow-up after primary S2: harder angles and bias failure boundary.
No controller retuning. Kept separate from the predeclared primary comparisons.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import json,time
from concurrent.futures import ProcessPoolExecutor,as_completed
import pandas as pd,numpy as np
from run_suite import worker,OUT
def main():
    items=[]
    def add(k):items.append((1000+len(items),k,{}))
    for angle in [-15,-10,10,15]:
        for x in [-.03,0,.03]:
            for c in ['fixed','eit','oracle']:
                for seed in range(800,803):add(dict(stage='S2',condition=c,offset=x,angle=angle,seed=seed))
    for bias in [-.03,-.04,-.05]:
        for angle in [-5,5]:
            for seed in range(810,815):add(dict(stage='S3',condition='oracle',offset=.027,angle=angle,seed=seed,bias=bias))
    (OUT/'exploratory_protocol.json').write_text(json.dumps({'reason':'Primary 5-degree S2 completed successfully for EIT/oracle; all primary +/-20mm bias cases succeeded. Explore larger angles and find a failure boundary without retuning. New seeds; report separately.','trials':items},indent=2))
    t=time.time();rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        fs=[pool.submit(worker,i) for i in items]
        for f in as_completed(fs):
            rows.append(f.result())
            if len(rows)%10==0:print('Extension',len(rows),'/',len(items),'seconds',round(time.time()-t),flush=True)
    pd.DataFrame(rows).sort_values('trial_id').to_csv(OUT/'exploratory_metrics.csv',index=False)
if __name__=='__main__':main()
