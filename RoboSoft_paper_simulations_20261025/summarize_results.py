#!/usr/bin/env python3
"""Summarize the paper's saved results, without running simulations."""
import csv,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent


def rows(relative):
    with (ROOT/relative).open() as f:return list(csv.DictReader(f))

def avg(data,key):
    values=[float(r[key]) for r in data if r.get(key) not in ('',None,'nan','None')]
    return sum(values)/len(values) if values else None

def report(data,condition,success,fields):
    out=[]
    for value in dict.fromkeys(r[condition] for r in data):
        part=[r for r in data if r[condition]==value]
        out.append({'condition':value,'n':len(part),'passes':sum(str(r[success]).lower()=='true' for r in part),**{k:avg(part,k) for k in fields}})
    return out


def main():
    sensor=json.loads((ROOT/'01_synthetic_eit_validation/results/eit_metrics.json').read_text())
    j=rows('02_joystick_attachment/main_attachment_comparison/analyzed_metrics.csv')
    b=[r for r in rows('03_ball_progress_after_push/progress_feedback/analyzed_metrics.csv') if r['case']=='forward_4N']
    attach=[r for r in rows('04_ball_moving_attachment/attachment_gain_sweep/analyzed_metrics.csv') if float(r['orientation_k'])==.3]
    loss=rows('05_ball_contact_loss_recovery/scheduled_vs_tactile_recovery/analyzed_metrics.csv')
    quality=rows('06_sensor_quality/bias_delay_sweep/analyzed_metrics.csv')
    cells=[]
    for bias in [-.005,0,.005]:
        for delay in [0,.1,.25,.5]:
            part=[r for r in quality if abs(float(r['progress_bias'])-bias)<1e-10 and abs(float(r['progress_delay'])-delay)<1e-10]
            assert len(part)==3
            cells.append({'bias_mm':bias*1000,'delay_ms':delay*1000,'mean_error_mm':avg(part,'late_error_mm'),'passes':sum(r['common_success']=='True' for r in part),'n':len(part)})
    output={'01_sensor_validation':sensor,
      '02_figure3a_joystick':report(j,'attachment','success',['peak_force_N','scored_orientation_error_deg']),
      '03_figure3b_forward_push':report(b,'progress_source','common_success',['late_error_mm','peak_force_N']),
      '04_figure3c_moving_attachment':report(attach,'attachment','common_success',['peak_pitch_deg','peak_force_N']),
      '05_figure3d_contact_loss':report(loss,'policy','success',['disconnected_motion_mm','peak_force_N']),
      '06_figure4_quality':cells}
    print(json.dumps(output,indent=2,allow_nan=False))


if __name__=='__main__':main()
