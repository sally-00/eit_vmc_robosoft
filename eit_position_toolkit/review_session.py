#!/usr/bin/env python3
"""Review saved position-training signal quality without connecting to hardware."""
import argparse
import json
from pathlib import Path
import numpy as np
from contact_signal_debug import signal_summary


def finite_json(value):
    if isinstance(value,np.ndarray):return finite_json(value.tolist())
    if isinstance(value,np.generic):return finite_json(value.item())
    if isinstance(value,float) and not np.isfinite(value):return None
    if isinstance(value,dict):return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [finite_json(v) for v in value]
    return value


def review_records(records, metadata):
    device=metadata.get('device',{})
    rail=metadata.get('eit_config',{}).get('eit_adc_peak_limit_v',2.047)
    touches=[]
    for record in records:
        base=signal_summary(record['baseline'],device=device,adc_peak_limit_v=rail)
        touch=signal_summary(record['contact'],record['baseline'],device,rail)
        usable=np.abs(base['voltage'])>1e-12
        noise=float(np.sqrt(np.mean(base['frame_noise_percent'][usable]**2))) if usable.any() else np.nan
        change=float(np.sqrt(np.mean(np.abs(touch['relative_percent'][usable])**2))) if usable.any() else np.nan
        clean=touch['usage_percent'][np.isfinite(touch['usage_percent'])]
        issues=[]
        if base['clipped'].any() or touch['clipped'].any():issues.append('clipped amplitudes are unreliable')
        if base['zero_channels'].any() or touch['zero_channels'].any():issues.append('zero readings present; inspect channel table')
        if not usable.all():issues.append('zero mean baseline channels will be excluded by fitting')
        if np.isfinite(noise) and noise>2:issues.append('baseline varies >2%; hold unloaded pad/wiring steady')
        if np.isfinite(change) and np.isfinite(noise) and change<=max(3*noise,1e-9):issues.append('touch change not clearly above baseline variation')
        touches.append({'point_id':record['point_id'],'repeat':record['repeat'],'xy_mm':record['xy_mm'],
            'baseline_noise_rms_percent':noise,'touch_change_rms_percent':change,
            'change_to_noise_ratio':change/noise if noise>0 else None,
            'baseline_clipped_channels':(np.flatnonzero(base['clipped'])+1).tolist(),
            'touch_clipped_channels':(np.flatnonzero(touch['clipped'])+1).tolist(),
            'zero_channels':(np.flatnonzero(base['zero_channels']|touch['zero_channels'])+1).tolist(),
            'peak_clean_sine_range_percent':float(clean.max()) if len(clean) else None,'issues':issues})
    return finite_json({'touch_count':len(touches),'touches_with_advisories':sum(bool(t['issues']) for t in touches),
        'note':'Advisories only; no samples rejected or changed. Range is a gain-dependent sine estimate from supplied v1.1.1 source. Within-capture variation during motion is not pure electronic noise.',
        'touches':touches})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True,type=Path)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    metadata=json.loads((a.session/'manifest.json').read_text())
    records=[json.loads(line) for line in (a.session/'touches.jsonl').read_text().splitlines() if line.strip()]
    report=review_records(records,metadata)
    output=a.output or a.session/'signal_quality.json'
    output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(f"{report['touches_with_advisories']}/{report['touch_count']} touches have signal advisories. Report: {output}")


if __name__=='__main__':main()
