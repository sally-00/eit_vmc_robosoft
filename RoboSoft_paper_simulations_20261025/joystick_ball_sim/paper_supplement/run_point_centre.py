"""Post-campaign matched centre baseline; original frozen campaign is preserved."""
from pathlib import Path
import sys, json, hashlib
from concurrent.futures import ProcessPoolExecutor, as_completed
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from trial import run_trial
OUT = Path(__file__).resolve().parent

def worker(cfg):
    return run_trial(cfg, OUT/'trials'/f"point_centre_{cfg['trial_id']:04d}.npz")

def main():
    protocol = json.loads((ROOT/'results/frozen_protocol.json').read_text())
    rows = []
    for original in protocol['trials']:
        if original['group'] == 'J2':
            cfg = dict(original, attachment='centre', group='J2_centre_supplement')
            rows.append(cfg)
    record = dict(note='Added after inspecting the original campaign to provide the missing matched point-controller centre baseline. No gain or criterion changes. Seeds repeat the original 27 J2 conditions; this is not an independent holdout.',
                  trials=rows, original_protocol_sha256=hashlib.sha256((ROOT/'results/frozen_protocol.json').read_bytes()).hexdigest(),
                  sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src').glob('*.py')})
    dest=OUT/'protocol.json'
    if dest.exists():
        assert json.loads(dest.read_text()) == record
    else:
        dest.write_text(json.dumps(record, indent=2))
    results=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        futures=[]
        for cfg in rows:
            path=OUT/'trials'/f"point_centre_{cfg['trial_id']:04d}.npz"
            if path.exists():
                import numpy as np
                with np.load(path) as data: results.append(json.loads(str(data['metrics'])))
            else:
                futures.append(pool.submit(worker,cfg))
        for future in as_completed(futures):
            result=future.result(); results.append(result)
            print(json.dumps({k:result[k] for k in ['trial_id','offset','gap','seed','success','orientation_error_deg','guard']}),flush=True)
    (OUT/'metrics.json').write_text(json.dumps(sorted(results,key=lambda r:r['trial_id']),indent=2))

if __name__=='__main__': main()
