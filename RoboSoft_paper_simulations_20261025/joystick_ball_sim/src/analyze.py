"""Trial-level summaries, including unfavorable and incomplete task outcomes."""
from collections import Counter
import json
import numpy as np
import pandas as pd
from scene import ROOT

OUT=ROOT/'results'


def main():
    protocol=json.loads((OUT/'frozen_protocol.json').read_text())
    rows=[]
    for cfg in protocol['trials']:
        path=OUT/'trials'/f"trial_{cfg['trial_id']:04d}.npz"
        if not path.exists():raise RuntimeError(f'Evaluation incomplete: {path}')
        d=np.load(path);row=json.loads(str(d['metrics']));tr=d['trace']
        col={name:i for i,name in enumerate(d['columns'].tolist())}
        t=tr[:,col['t']]
        first=row.get('first_engagement_s')
        mask=t<=first if first is not None else np.ones(len(t),dtype=bool)
        row['pre_engagement_peak_N']=float(tr[mask,col['force_N']].max()) if mask.any() else 0.
        events=json.loads(str(d['events']))
        reconnects=[e for e in events if e['to']=='engage'][1:]
        peaks=[]
        for e in reconnects:
            mask=(t>=e['t'])&(t<=e['t']+.5)
            if mask.any():peaks.append(float(tr[mask,col['force_N']].max()))
        row['post_reconnection_peak_N']=max(peaks) if peaks else None
        row['reconnections']=len(reconnects)
        row['false_reconnections']=sum(e['force_N']<.05 for e in reconnects)
        row['engagement_delay_s']=first-row['first_contact_s'] if first is not None and row['first_contact_s'] is not None else None
        row['policy']=row['switching']+'_'+row['attachment']+('_point' if row['mechanism']=='point' else '')
        row['translation_distance_error_mm']=abs(row['final_ball_x']-tr[0,col['ball_x']]-cfg['distance'])*1000 if cfg['task']=='ball' else None
        if row['guard']=='force_or_rim_or_speed':
            causes=[]
            if row['peak_force_N']>15:causes.append('pad_force')
            if row['peak_rim_N']>1:causes.append('rim_contact')
            if row['peak_joint_speed']>5:causes.append('joint_speed')
            row['termination_cause']='+'.join(causes)
        else:
            row['termination_cause']=row['guard'] or 'completed'
        # Missing joystick scoring windows were encoded as 180 by the worker;
        # expose them as unavailable while retaining every trial in pass counts.
        row['scored_orientation_error_deg']=row['orientation_error_deg'] if row['orientation_error_deg']<179 else None
        rows.append(row)
    df=pd.DataFrame(rows)
    if 'rate' not in df:df['rate']=20.
    df['rate']=df['rate'].fillna(20.)
    for col,default in [('sensor_shift',False),('radius',.025),('stick_k',.15),('interrupt',False),('resistance',False)]:
        if col not in df:df[col]=default
        df[col]=df[col].fillna(default)
    df.to_csv(OUT/'analyzed_metrics.csv',index=False)
    def summarize(s):
        return dict(n=len(s),passes=int(s.success.sum()),physical_task_passes=int(s.task_success.sum()),
                    completions=int(s.completed.sum()),premature_episodes=int((s.premature_engagements>0).sum()),
                    mean_peak_force_N=float(s.peak_force_N.mean()),max_force_N=float(s.peak_force_N.max()),
                    median_time_s=float(s.duration_s.median()),mean_time_s=float(s.duration_s.mean()),
                    mean_endpoint_error_mm=float(s.target_error.mean()) if (s.task=='ball').all() else None,
                    mean_translation_distance_error_mm=float(s.translation_distance_error_mm.mean()) if (s.task=='ball').all() else None,
                    mean_scored_orientation_error_deg=float(s.scored_orientation_error_deg.mean()) if (s.task=='joystick').all() and s.scored_orientation_error_deg.notna().any() else None,
                    scored_orientation_trials=int(s.scored_orientation_error_deg.notna().sum()),
                    mean_disconnected_motion_mm=float(s.disconnected_motion_mm.mean()),
                    mean_loss_response_s=float(s.loss_response_s.mean()) if s.loss_response_s.notna().any() else None,
                    mean_stopping_latency_s=float(s.stopping_latency_s.mean()) if s.stopping_latency_s.notna().any() else None,
                    stopped_in_window=int(s.stopping_latency_s.notna().sum()),
                    mean_strokes=float(s.strokes.mean()),total_reconnections=int(s.reconnections.sum()),
                    false_reconnections=int(s.false_reconnections.sum()),
                    rolling_passes=int(s.rolling_success.fillna(False).sum()),
                    max_rolling_distance_error_mm=float(s.rolling_distance_error_mm.max()),
                    mean_location_median_mm=float(s.location_median_mm.mean()) if s.location_median_mm.notna().any() else None,
                    max_rebase_error_m=float(s.max_rebase_error_m.max()),
                    guard_counts=dict(Counter(s.guard.fillna('').replace('', 'none'))))
    summary={'total_trials':len(df),'physical_steps':int(df.physical_steps.sum()),
             'max_physical_steps':protocol['max_physical_steps'],
             'all':summarize(df),'groups':{},'primary':{},'loss':{},'multistroke':{}}
    for group,s in df.groupby('group'):
        summary['groups'][group]=summarize(s)
    for task,groups in [('joystick',['J1','J2']),('ball',['B1','B2'])]:
        summary['primary'][task]={p:summarize(s) for p,s in df[df.group.isin(groups)].groupby('policy')}
    for key,group in [('loss','B4'),('multistroke','B3')]:
        summary[key]={p:summarize(s) for p,s in df[df.group==group].groupby('policy')}
    paired={}
    for task,groups in [('joystick',['J1','J2']),('ball',['B1','B2'])]:
        s=df[df.group.isin(groups)&(df.switching=='event')&(df.attachment=='eit')]
        a=s[s.mechanism=='mechanism'];b=s[s.mechanism=='point']
        p=a.merge(b,on=['task','seed','gap','offset'],suffixes=('_mechanism','_point'),validate='one_to_one')
        paired[task]={'n_pairs':len(p),'mechanism_only_passes':int((p.success_mechanism&~p.success_point).sum()),
                      'point_only_passes':int((~p.success_mechanism&p.success_point).sum()),
                      'both_pass':int((p.success_mechanism&p.success_point).sum()),
                      'mean_error_mechanism':float(p.target_error_mechanism.mean()),
                      'mean_error_point':float(p.target_error_point.mean())}
    summary['paired_mechanism_vs_point']=paired
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    print(json.dumps({'total':summary['total_trials'],'steps':summary['physical_steps'],
                      'primary':summary['primary'],'loss':summary['loss']},indent=2))


if __name__=='__main__':main()
