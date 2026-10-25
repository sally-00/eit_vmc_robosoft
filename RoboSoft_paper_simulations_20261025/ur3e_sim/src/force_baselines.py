"""Post-hoc held-out force-decoder baselines; do not retune executed controllers."""
import json
import numpy as np
from sklearn.isotonic import IsotonicRegression
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler,PolynomialFeatures
from sklearn.linear_model import Ridge
from experiments import OUT
def main():
    data={n:np.load(OUT/f'eit_{n}.npz') for n in ['train','validation','test','shift']}
    pred={n:np.load(OUT/f'eit_{n}_predictions.npz')['pred'] for n in data}
    mag={n:np.linalg.norm(d['X'],axis=1) for n,d in data.items()}
    base=IsotonicRegression(out_of_bounds='clip').fit(mag['train'],data['train']['Y'][:,2])
    # Training locations are supervised labels, test locations are decoder outputs.
    features={n:np.c_[mag[n],(data[n]['Y'][:,:2] if n=='train' else pred[n][:,:2])/.03] for n in data}
    choices=[]
    for alpha in [.01,1,10]:
        m=make_pipeline(PolynomialFeatures(3),StandardScaler(),Ridge(alpha=alpha)).fit(features['train'],data['train']['Y'][:,2])
        error=np.mean(abs(np.clip(m.predict(features['validation']),0,10)-data['validation']['Y'][:,2]));choices.append((error,alpha,m))
    _,alpha,m=min(choices,key=lambda x:x[0]);results={'note':'Post-hoc calibration comparison; controller results retain the original multi-output tree estimator. Conditioned model trains with labelled positions and uses estimated positions at evaluation.','selected_ridge_alpha':alpha,'datasets':{}}
    for n in ['validation','test','shift']:
        y=data[n]['Y'][:,2]
        results['datasets'][n]={'magnitude_isotonic_mae_N':float(np.mean(abs(base.predict(mag[n])-y))),
            'location_conditioned_polynomial_mae_N':float(np.mean(abs(np.clip(m.predict(features[n]),0,10)-y))),
            'joint_tree_mae_N':float(np.mean(abs(pred[n][:,2]-y)))}
    (OUT/'force_decoder_baselines.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
if __name__=='__main__':main()
