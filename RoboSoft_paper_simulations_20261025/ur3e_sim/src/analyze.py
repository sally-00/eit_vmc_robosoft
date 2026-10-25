"""Episode-level summaries. Repeated time samples are never statistical replicates."""
from pathlib import Path
import json,hashlib
import numpy as np,pandas as pd
from experiments import OUT

def augment(df):
    records=[]
    for _,r in df.iterrows():
        tr=np.load(OUT/'trial_data'/f'trial_{int(r.trial_id):04d}.npz')['trace'];e=tr[:,0]-r.latch_time_s
        early=tr[:,0]<=r.latch_time_s;late=(e>=3.8)&(e<=4.4);ret=(e>=4)&(e<=4.4)
        records.append({'trial_id':int(r.trial_id),'whole_episode_peak_N':float(tr[:,1].max()),'pre_latch_peak_N':float(tr[early,1].max()) if early.any() else np.nan,
            'late_force_mae_N':float(np.mean(abs(tr[late,1]-r.target_N))) if late.any() else np.nan,
            'return_rotation_error_deg':float(np.mean(abs(np.rad2deg(tr[ret,3])))) if ret.any() else np.nan,
            'full_task_slip_mm':float(tr[:,6].max()*1000),
            'contact_migration_max_mm':float(np.nanmax(tr[(e>=.8)&(e<=2.8),5])*1000) if ((e>=.8)&(e<=2.8)).any() else np.nan})
    return df.merge(pd.DataFrame(records),on='trial_id')

def main():
    p=augment(pd.read_csv(OUT/'trial_metrics.csv'));p.to_csv(OUT/'analyzed_metrics.csv',index=False)
    ex=augment(pd.read_csv(OUT/'exploratory_metrics.csv'));ex.to_csv(OUT/'analyzed_exploratory.csv',index=False)
    primary=p[p.stage=='S2'];result={'primary_trials':len(p),'exploratory_trials':len(ex),'S2':{},'S1':{},'S4':{},'S3':{}}
    for c in ['fixed','eit','oracle']:
        g=primary[primary.condition==c]
        result['S2'][c]={'successes':int(g.success.sum()),'trials':len(g),'mean_abs_orientation_error_deg':float(abs(g.achieved_deg-g.angle_deg).mean()),
            'max_anchor_drift_mm':float(g.anchor_drift_mm.max()),'mean_force_peak_N':float(g.peak_force_N.mean()),'max_contact_migration_mm':float(g.contact_migration_max_mm.max()),
            'minimum_contact_fraction':float(g.contact_fraction.min()),'max_slip_mm':float(g.slip_mm.max())}
        g=p[(p.stage=='S1')&(p.condition==c)&(p.offset_mm!=0)]
        result['S1'][c]={'offcentre_mean_peak_rotation_deg':float(g.rotation_abs_deg.mean())}
    for c in ['nominal','force_eit','force_oracle']:
        g=p[(p.stage=='S4')&(p.condition==c)]
        result['S4'][c]={'mean_primary_force_mae_N':float(g.force_mae_N.mean()),'mean_late_force_mae_N':float(g.late_force_mae_N.mean()),'max_force_N':float(g.whole_episode_peak_N.max())}
    for rate in [1,5,10,20]:
        g=p[(p.stage=='S3')&(p.condition=='eit')&(p.rate_hz==rate)&(p.delay_ms==0)&(p.coarse_mm==0)&(p.noise==0)&p.mu.isna()&(~p['shift'])]
        result['S3'][f'rate_{rate}']={'n':len(g),'success':int(g.success.sum()),'mean_latch_time_s':float(g.latch_time_s.mean()),'pre_latch_peak_N':float(g.pre_latch_peak_N.max())}
    result['exploratory_angle_success']=ex[ex.stage=='S2'].assign(magnitude=lambda x:abs(x.angle_deg)).groupby(['magnitude','condition']).success.agg(['sum','count']).reset_index().to_dict('records')
    result['exploratory_bias_success']=ex[ex.stage=='S3'].groupby(['bias_mm','angle_deg']).success.agg(['sum','count']).reset_index().to_dict('records')
    result['no_guard_abort_primary']=bool(not p.guard.any());result['torque_saturated_steps_primary']=int(p.torque_saturated_steps.sum())
    protocol=json.loads((OUT/'frozen_protocol.json').read_text())
    result['frozen_source_matches']={name:hashlib.sha256((OUT.parent/'src'/name).read_bytes()).hexdigest()==sha for name,sha in protocol['source_sha256'].items()}
    (OUT/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
