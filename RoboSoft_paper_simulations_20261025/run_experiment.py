#!/usr/bin/env python3
"""Rerun selected archived simulations into a NEW directory, preserving evidence."""
import argparse
from datetime import datetime,timezone
import importlib
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--experiment',help='01 through 08, in paper order')
    p.add_argument('--subset',help='Optional subset name shown by --list')
    p.add_argument('--limit',type=int,help='Run only the first N selected trials')
    p.add_argument('--output',type=Path,help='New output directory (must not exist)')
    p.add_argument('--list',action='store_true')
    a=p.parse_args();index=json.loads((ROOT/'EXPERIMENT_INDEX.json').read_text())
    if a.list:
        for e in index:print(e['folder'],e['trial_count'],'saved trajectories;',e['subsets'])
        return
    selected=[e for e in index if a.experiment and e['folder'].startswith(a.experiment+'_')]
    if len(selected)!=1:p.error('Choose --experiment 01 through 08, or --list')
    if a.limit is not None and a.limit<1:p.error('--limit must be positive')
    entry=selected[0]
    os.environ.setdefault('OPENBLAS_NUM_THREADS','1');os.environ.setdefault('OMP_NUM_THREADS','1')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output=(a.output or ROOT/'reruns'/(entry['folder']+'_'+stamp)).resolve()
    # Reject all destinations within frozen evidence/source directories.
    protected=[ROOT/e['folder'] for e in index]+[ROOT/'paper',ROOT/'joystick_ball_sim',ROOT/'ur3e_sim',ROOT/'provenance']
    if any(output==d or d in output.parents for d in protected):p.error('Output must be separate from the archived evidence and source')
    if entry['folder'].startswith('01_'):
        output.mkdir(parents=True,exist_ok=False)
        sys.path.insert(0,str(ROOT/'ur3e_sim/src'))
        sensor=importlib.import_module('eit');sensor.ROOT=output
        print('Regenerating synthetic datasets and decoder; output:',output,flush=True)
        sensor.run_training()
        return
    records=json.loads((ROOT/entry['folder']/'trial_index.json').read_text())
    if a.subset:records=[r for r in records if r['subset']==a.subset]
    if not records:p.error('No trials match this subset; see --list')
    if a.limit:records=records[:a.limit]
    output.mkdir(parents=True,exist_ok=False);(output/'trials').mkdir()
    (output/'requested_trials.json').write_text(json.dumps(records,indent=2)+'\n')
    sys.path.insert(0,str(ROOT/'joystick_ball_sim/src'))
    sys.path.insert(0,str(ROOT/'joystick_ball_sim/followup_20261022/src'))
    functions={};results=[]
    for i,r in enumerate(records,1):
        if r['engine'] not in functions:
            module='trial' if r['engine']=='original' else 'trial_followup'
            functions[r['engine']]=importlib.import_module(module).run_trial
        print(f"[{i}/{len(records)}] {r['uid']} / {r['subset']}",flush=True)
        result=functions[r['engine']](dict(r['config']),output/'trials'/(r['uid']+'.npz'))
        results.append({'uid':r['uid'],'metrics':result})
        (output/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
    print('Finished. Original evidence was not changed. New results:',output)
    print('Use the archived analysis/scoring code for derived metrics; raw worker metrics are not all Figure 3/4 analysis columns.')


if __name__=='__main__':main()
