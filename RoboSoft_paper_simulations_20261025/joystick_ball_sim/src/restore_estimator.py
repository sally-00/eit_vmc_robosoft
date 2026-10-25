"""Recover the truncated copied checkpoint from unchanged saved training arrays."""
import hashlib
import json
from pathlib import Path
import joblib
import numpy as np
from sklearn.ensemble import ExtraTreesRegressor
from scene import ROOT, BASE


def main():
    out = ROOT/'results'; out.mkdir(exist_ok=True)
    manifest = json.loads((BASE/'MANIFEST.json').read_text())
    def sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    train = np.load(BASE/'results/eit_train.npz')
    model = ExtraTreesRegressor(n_estimators=100, min_samples_leaf=1,
                               max_features=1., random_state=51, n_jobs=1)
    model.fit(train['X'], train['Y']/[.03, .03, 5.])
    joblib.dump(model, out/'eit_estimator_restored.joblib')
    record = {'reason': 'Original copied checkpoint raises EOF while reading an array.',
              'original_checkpoint_sha256': sha(BASE/'results/eit_estimator.joblib'),
              'manifest_checkpoint_sha256': manifest['results/eit_estimator.joblib'],
              'restored_checkpoint_sha256': sha(out/'eit_estimator_restored.joblib'),
              'training_data_sha256': sha(BASE/'results/eit_train.npz'),
              'manifest_training_data_sha256': manifest['results/eit_train.npz'],
              'comparison': {}}
    for split in ['train', 'validation', 'test', 'shift']:
        data = np.load(BASE/f'results/eit_{split}.npz')
        pred = model.predict(data['X'])*[.03, .03, 5.]
        saved = np.load(BASE/f'results/eit_{split}_predictions.npz')['pred']
        record['comparison'][split] = {'max_absolute_prediction_difference': float(np.max(abs(pred-saved)))}
    (out/'estimator_restoration.json').write_text(json.dumps(record, indent=2))
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
