"""Checks of geometry, force mapping, transmission, and robot time resolution."""
import json
import numpy as np
import mujoco
from scene import ROOT, make_scene
from trial import run_trial
from electrical import EIT


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def main():
    checks={}
    m,d,_,_=make_scene(dict(task='ball',gap=.006))
    body=m.body('tool').id
    jp=np.zeros((3,m.nv));jw=np.zeros_like(jp);jc=np.zeros_like(jp)
    mujoco.mj_jac(m,d,jp,jw,d.xpos[body],body)
    R=d.xmat[body].reshape(3,3)
    jacerr=0.;tauerr=0.;powererr=0.
    for offset in [-.03,0.,.03]:
        r=np.array([offset,.01,.0335]);pc=d.xpos[body]+R@r
        mujoco.mj_jac(m,d,jc,None,pc,body)
        explicit=jp-skew(R@r)@jw
        jacerr=max(jacerr,float(np.max(abs(jc-explicit))))
        f=np.array([.8,.2,-3.]);tauerr=max(tauerr,float(np.max(abs(jc.T@f-explicit.T@f))))
        for G in [np.array([2.,0,0]),np.array([.092*np.cos(.2),0,-.092*np.sin(.2)])]:
            powererr=max(powererr,abs(float(f@(G*.03)-.03*(G@f))))
    checks['contact_frame_jacobian_max_difference']=jacerr
    checks['equivalent_contact_frame_torque_max_difference_Nm']=tauerr
    checks['virtual_transmission_power_residual_W']=powererr
    base=EIT(.0048,angle=.006);fine=EIT(.0028,angle=.006)
    a=base.solve([.02,.007],3)-base.solve()
    b=fine.solve([.02,.007],3)-fine.solve()
    checks['forward_4p8_vs_2p8mm_relative_contact_signal_difference']=float(np.linalg.norm(a-b)/np.linalg.norm(b))
    refinements=[]
    for task in ['joystick','ball']:
        rows=[]
        for dt in [.001,.0005]:
            cfg=dict(task=task,attachment='eit',switching='event',mechanism='mechanism',
                     seed=104,gap=.006,offset=0.,distance=.025,dt=dt,
                     nominal_time=1.,horizon=16. if task=='joystick' else 28.)
            row=run_trial(cfg,ROOT/'verification'/f'{task}_{dt}.npz');rows.append(row)
        refinements.append({'task':task,'step_sizes_s':[.001,.0005],
                            'successes':[r['success'] for r in rows],
                            'target_error_units':'degrees' if task=='joystick' else 'mm',
                            'target_error_difference':abs(rows[0]['target_error']-rows[1]['target_error']),
                            'peak_force_difference_N':abs(rows[0]['peak_force_N']-rows[1]['peak_force_N'])})
    checks['time_step_refinement']=refinements
    (ROOT/'results/verification.json').write_text(json.dumps(checks,indent=2))
    print(json.dumps(checks,indent=2))


if __name__=='__main__':main()
