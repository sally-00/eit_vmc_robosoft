"""Meaningful numerical checks; run independently of final comparison trials."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import numpy as np,json
import mujoco
from robot import make_model,ik
from experiments import Experiment,P0,R0,OUT

def main():
    m,xml=make_model();d=mujoco.MjData(m);d.qpos[:]=ik(m,P0,R0);mujoco.mj_forward(m,d)
    bid=m.body('tool').id;point=d.xpos[bid]+d.xmat[bid].reshape(3,3)@np.array([.027,0,.0335])
    jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jac(m,d,jp,jr,point,bid)
    q=d.qpos.copy();fd=np.zeros_like(jp);eps=1e-6
    for j in range(6):
        d.qpos[:]=q;d.qpos[j]+=eps;mujoco.mj_forward(m,d);p=d.xpos[bid]+d.xmat[bid].reshape(3,3)@np.array([.027,0,.0335]);fd[:,j]=(p-point)/eps
    checks={'point_jacobian_max_absolute_error':float(abs(fd-jp).max()),'mujoco_version':mujoco.__version__}
    assert checks['point_jacobian_max_absolute_error']<1e-5
    rows=[]
    for c in ['fixed','eit','oracle']:
        for dt in [.001,.0005]:
            row,tr=Experiment(dt=dt).run(condition=c,offset=.03,angle=5,seed=750)
            row['dt']=dt;rows.append(row)
    checks['step_refinement']=rows
    # Guard against large numerical differences in the actual task metrics.
    checks['maximum_step_change_rotation_deg']=max(abs(rows[i]['achieved_deg']-rows[i+1]['achieved_deg']) for i in range(0,len(rows),2))
    checks['maximum_step_change_peak_force_N']=max(abs(rows[i]['peak_force_N']-rows[i+1]['peak_force_N']) for i in range(0,len(rows),2))
    (OUT/'verification.json').write_text(json.dumps(checks,indent=2));print(json.dumps(checks,indent=2))
if __name__=='__main__':main()
