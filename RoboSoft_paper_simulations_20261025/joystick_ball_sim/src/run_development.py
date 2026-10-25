"""Small development set, excluded from the frozen evaluation schedule."""
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
from pathlib import Path
from scene import ROOT
from trial import run_trial


def worker(item):
    ident, cfg = item
    return run_trial(cfg, ROOT/'development'/f'dev_{ident:03d}.npz')


def main():
    items = []
    def add(**kw):
        cfg = dict(task='joystick', attachment='eit', switching='event', mechanism='mechanism',
                   seed=101, gap=.006, offset=0., distance=.025, nominal_time=1.0)
        cfg.update(kw); items.append((len(items), cfg))
    for offset in [-.025, .025]:
        for attachment in ['centre', 'eit', 'oracle']:
            add(offset=offset, attachment=attachment)
        add(offset=offset, mechanism='point')
    for offset in [-.015, .015]:
        for attachment in ['centre', 'eit', 'oracle']:
            add(task='ball', offset=offset, attachment=attachment)
        add(task='ball', offset=offset, mechanism='point')
    for gap in [.002, .014]:
        for switching in ['timed', 'event']:
            add(gap=gap, switching=switching)
    for switching in ['timed', 'event']:
        add(task='ball', distance=.08, horizon=40., switching=switching)
        add(task='ball', distance=.06, horizon=35., switching=switching, interrupt=True)
    results = []
    with ProcessPoolExecutor(max_workers=4) as executor:
        futures = {executor.submit(worker, x): x[0] for x in items}
        for future in as_completed(futures):
            row = future.result(); row['development_id'] = futures[future]; results.append(row)
            print(json.dumps({k: row.get(k) for k in ['development_id', 'task', 'attachment',
                  'mechanism', 'switching', 'gap', 'offset', 'distance', 'interrupt',
                  'success', 'guard', 'target_error', 'peak_force_N', 'strokes',
                  'recoveries', 'duration_s', 'premature_engagements']}), flush=True)
    (ROOT/'development/metrics.json').write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
