#!/usr/bin/env python3
"""Verify file hashes and optionally every saved numerical trajectory."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--traces',action='store_true',help='Also inspect NPZ configs, state shapes and finite values (requires NumPy)')
    a=p.parse_args()
    manifest=json.loads((ROOT/'SHA256_MANIFEST.json').read_text())
    for rel,expected in manifest.items():
        path=ROOT/rel
        if not path.is_file():raise RuntimeError('Missing file: '+rel)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise RuntimeError('Changed file: '+rel)
    sources=json.loads((ROOT/'SOURCE_HASH_AUDIT.json').read_text())
    for record in sources:
        if record['status']=='verified':
            actual=hashlib.sha256((ROOT/record['bundled_path']).read_bytes()).hexdigest()
            if actual!=record['expected_sha256']:raise RuntimeError('Frozen source mismatch: '+record['bundled_path'])
    print(f'Verified {len(manifest)} files and all archived simulator/runtime hashes.')
    index=json.loads((ROOT/'EXPERIMENT_INDEX.json').read_text());count=0
    for e in index:
        records=json.loads((ROOT/e['folder']/'trial_index.json').read_text())
        assert len(records)==e['trial_count']
        for r in records:
            path=ROOT/r['trace']
            if not path.is_file():raise RuntimeError('Missing trajectory: '+str(path))
            if a.traces:
                import numpy as np
                with np.load(path,allow_pickle=False) as z:
                    assert json.loads(str(z['config']))==r['config'],r['uid']
                    trace=z['trace'];names=z['columns'].tolist()
                    assert trace.ndim==2 and trace.shape[1]==len(names),r['uid']
                    assert np.isfinite(z['qpos']).all(),r['uid']
                    assert np.all(np.diff(trace[:,names.index('t')])>0),r['uid']
                    for i,name in enumerate(names):
                        # Contact position is undefined while the ball is detached.
                        if name!='true_contact_x':assert np.isfinite(trace[:,i]).all(),(r['uid'],name)
            count+=1
    print(f'Verified {count} indexed saved trajectories'+(' including numerical contents.' if a.traces else '.'))
    print('Synthetic results only. This does not execute or validate a physical robot.')


if __name__=='__main__':main()
