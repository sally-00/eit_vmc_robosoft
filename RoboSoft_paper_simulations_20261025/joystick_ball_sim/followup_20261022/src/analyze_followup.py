"""All-attempt analysis and saved-trace audit; no reruns or controller changes."""
from pathlib import Path
import json,hashlib,sys
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]

def grouped(df,keys):
    out=[]
    for key,d in df.groupby(keys,dropna=False,sort=False):
        if not isinstance(key,tuple):key=(key,)
        row=dict(zip(keys,key));row.update(n=len(d),passes=int(d.common_success.sum()),
            scored=int(d.late_window_available.sum()),guard_stops=int((d.guard!='').sum()))
        for field in ['late_error_mm','late_speed_mm_s','peak_force_N','peak_rim_N','peak_pitch_deg',
                      'mean_pitch_deg','progress_mae_mm','integrated_error_mm_s','maximum_overshoot_mm',
                      'top_slip_mm','bottom_slip_mm','contact_fraction','settled_age_s']:
            v=d[field].dropna()
            row[field]=float(v.mean()) if len(v) else None
        out.append(row)
    return out

def main():
    out=ROOT/'evaluation';protocol=json.loads((out/'protocol.json').read_text())
    df=pd.DataFrame(json.loads((out/'metrics.json').read_text()))
    assert len(df)==len(protocol['trials']) and df.trial_id.nunique()==len(df)
    df['dynamics_label']=np.where(df.mechanism=='point','prescribed',np.where(df.reaction_scale.fillna(1)==0,'no_reaction','carriage'))
    df.to_csv(out/'analyzed_metrics.csv',index=False)
    summary={
        'feedback':grouped(df[df.group=='feedback'],['case','progress_source']),
        'feedback_point':grouped(df[df.group=='feedback_point'],['progress_source']),
        'attachment':grouped(df[df.group=='attachment'],['orientation_k','attachment']),
        'attachment_by_offset':grouped(df[df.group=='attachment'],['offset','orientation_k','attachment']),
        'boundary':grouped(df[df.group=='attachment_boundary'],['attachment']),
        'dynamics':grouped(df[df.group=='dynamics'],['case','dynamics_label']),
        'quality':grouped(df[df.group=='quality'],['progress_bias','progress_delay']),
        'dropout':grouped(df[df.group=='dropout'],['dropout_duration'])}
    pairs=[]
    for group in ['feedback','feedback_point']:
        d=df[df.group==group]
        for case,part in d.groupby('case'):
            w=part.pivot(index=['offset','seed'],columns='progress_source',values='late_error_mm')
            common=w[['odometry','eit']].dropna()
            pairs.append(dict(group=group,case=case,paired_available=len(common),
                eit_lower=int((common.eit<common.odometry).sum()),
                mean_reduction_mm=float((common.odometry-common.eit).mean()) if len(common) else None))
    summary['paired_feedback']=pairs
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    issues=[];steps=0
    for cfg in protocol['trials']:
        with np.load(out/'trials'/f"trial_{cfg['trial_id']:04d}.npz") as z:
            c=json.loads(str(z['config']));m=json.loads(str(z['metrics']));tr=z['trace']
            if c!=cfg:issues.append([cfg['trial_id'],'config_mismatch'])
            # True contact coordinate is deliberately NaN outside contact.
            if not np.isfinite(z['qpos']).all() or not np.isfinite(np.delete(tr,31,axis=1)).all():
                issues.append([cfg['trial_id'],'nonfinite'])
            if np.any(np.diff(tr[:,0])<=0):issues.append([cfg['trial_id'],'time_order'])
            if m['common_success'] and not m['late_window_available']:issues.append([cfg['trial_id'],'scoring'])
            steps+=m['physical_steps']
    for path,h in protocol['source_sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=h:issues.append([path,'source_changed'])
    audit=dict(attempts=len(df),physical_steps=steps,issues=issues,
               note='Contact position is undefined outside contact. All other saved states and trace values are finite.')
    (out/'audit.json').write_text(json.dumps(audit,indent=2));print(json.dumps(audit,indent=2))
    for group in ['feedback','feedback_point','attachment','dynamics','quality','dropout']:
        print('\n'+group)
        for r in summary[group]:
            print({k:v for k,v in r.items() if k in ['case','progress_source','orientation_k','attachment','dynamics_label','progress_bias','progress_delay','dropout_duration','n','passes','scored','late_error_mm','peak_pitch_deg','peak_force_N']})
if __name__=='__main__':main()
