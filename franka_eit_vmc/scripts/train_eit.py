#!/usr/bin/env python3
"""Train a portable position-only model from this package or the existing GUI sessions."""
import argparse,json,hashlib
from pathlib import Path
from franka_eit_tools.position_model import fit_samples

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args()
    if args.output.suffix!='.npz':p.error('--output must end in .npz')
    meta=json.loads((args.session/'manifest.json').read_text())
    candidates=[args.session/'touches.jsonl',args.session/'samples.jsonl',args.session/'records.jsonl']
    path=next((x for x in candidates if x.exists()),None)
    if path is None:p.error('No touches.jsonl, samples.jsonl or records.jsonl in session')
    if meta.get('demo') is not False:p.error('Physical export requires a session explicitly captured from hardware (demo=false)')
    records=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    meta['source_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    model,report=fit_samples(records,meta)
    args.output.parent.mkdir(parents=True,exist_ok=True);model.save(args.output)
    args.output.with_suffix('.json').write_text(json.dumps({'evaluation':report,'metadata':model.metadata,'sha256':hashlib.sha256(args.output.read_bytes()).hexdigest()},indent=2)+'\n')
    print(json.dumps(report,indent=2));print('Exported:',args.output)
if __name__=='__main__':main()
