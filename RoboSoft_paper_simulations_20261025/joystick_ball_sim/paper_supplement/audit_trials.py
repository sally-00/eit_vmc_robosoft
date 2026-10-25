from pathlib import Path
import hashlib,json,sys
import numpy as np
root=Path('/media/yi/Yi Cam data/09_EIT_vmc/joystick_ball_sim')
folder=root/'paper_supplement'
p=json.loads((folder/'protocol.json').read_text())
original=json.loads((root/'results/frozen_protocol.json').read_text())
refs={c['trial_id']:c for c in original['trials']}
issues=[];steps=0
for cfg in p['trials']:
    expected=dict(refs[cfg['trial_id']],attachment='centre',group='J2_centre_supplement')
    if cfg!=expected: issues.append(f"config mismatch {cfg['trial_id']}")
    with np.load(folder/'trials'/f"point_centre_{cfg['trial_id']:04d}.npz") as data:
        if json.loads(str(data['config']))!=cfg:issues.append('saved config mismatch')
        for key in ['trace','qpos','mocap']:
            if not np.isfinite(data[key]).all():issues.append('nonfinite '+key)
        metrics=json.loads(str(data['metrics']))
        steps+=metrics['physical_steps']
for name,sha in p['sources'].items():
    if hashlib.sha256((root/'src'/name).read_bytes()).hexdigest()!=sha:issues.append('source changed '+name)
archive=root.parent/'RoboSoft_EIT_VMC_Draft/revisions/before_task_results_20261021/manuscript.tex'
key='RoboSoft_EIT_VMC_Draft/manuscript.tex'
archive_matches=hashlib.sha256(archive.read_bytes()).hexdigest()==original['source_sha256'][key]
if not archive_matches:issues.append('archived pre-edit manuscript differs from original frozen source')
record=dict(trials=len(p['trials']),physical_steps=steps,issues=issues,
            archived_manuscript_matches_original_frozen_hash=archive_matches,
            note='Original campaign unchanged; its frozen manuscript dependency is preserved in the draft revisions folder. The current manuscript is a later presentation revision.')
(folder/'audit.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2))
assert not issues
