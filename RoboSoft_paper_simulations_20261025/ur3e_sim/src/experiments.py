"""Executed S0--S4 protocols. All data are synthetic; no real robot commands."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from pathlib import Path
import json,time,argparse
import numpy as np
import pandas as pd
import mujoco,joblib
from scipy.spatial.transform import Rotation
from scipy.integrate import solve_ivp
from robot import make_model,ik,LIMITS
from eit import EIT,session,measure
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results'
R0=np.diag([1.,-1.,-1.]);P0=np.array([.3,0,.25]);DT=.001

def s0():
    def run(r,rhat,dt=.001):
        t=np.arange(0,4+dt/2,dt)
        def fun(t,y):
            x,phi,xd,pd=y;F=5*np.clip((t-.5)/.2,0,1) if t<2.5 else 0
            fv=-200*(x-rhat*phi)-30*(xd-rhat*pd)
            return [xd,pd,(fv+F)/2,(-rhat*fv-2*phi-.3*pd-r*F)/.02]
        # Fixed-step RK4 exactly as in original draft.
        y=np.zeros((len(t),4))
        for i in range(len(t)-1):
            a=np.array(fun(t[i],y[i]));b=np.array(fun(t[i]+dt/2,y[i]+dt*a/2));c=np.array(fun(t[i]+dt/2,y[i]+dt*b/2));d=np.array(fun(t[i]+dt,y[i]+dt*c))
            y[i+1]=y[i]+dt*(a+2*b+2*c+d)/6
        return t,y,y[:,0]-r*y[:,1]
    metrics={};allrows=[]
    for r in [.06,.03]:
        for name,rhat in [('fixed',0),('oracle',r)]:
            t,y,c=run(r,rhat);h=(t>=2.2)&(t<=2.45)
            metrics[f'{r}_{name}']={'displacement_mm':float(c[h].mean()*1000),'rotation_deg':float(np.rad2deg(y[h,1].mean())),'analytic_displacement_mm':1000*(5/200+(rhat-r)**2*5/2)}
            allrows.append(np.c_[t,y,c,np.full(len(t),r),np.full(len(t),rhat)])
    np.savez_compressed(OUT/'s0_traces.npz',traces=np.array(allrows))
    t,y,c=run(.06,.06);tt,yy,cc=run(.06,.06,.0005)
    metrics['half_step_max_contact_mm']=float(np.max(abs(c-cc[::2]))*1000)
    (OUT/'s0_metrics.json').write_text(json.dumps(metrics,indent=2))

class Experiment:
    def __init__(self,spring=0.,mu=.6,soft=.008,dt=DT):
        self.m,self.xml=make_model(spring,mu,soft,dt);self.dt=dt;self.q0=ik(self.m,P0,R0)
        self.eit=EIT(.0035,angle=.004);self.est=joblib.load(OUT/'eit_estimator.joblib')
        self.bid=self.m.body('tool').id;self.sid=self.m.site('sensor').id;self.support=self.m.body('support').id
        self.front=self.m.geom('front').id;self.housing=self.m.geom('housing').id;self.tip=self.m.geom('tip').id

    def contact(self,d):
        force=0.;weighted=np.zeros(3);slip=0.;f=np.zeros(6)
        R=d.xmat[self.bid].reshape(3,3);o=d.xpos[self.bid]
        for i in range(d.ncon):
            c=d.contact[i]
            if self.tip not in [c.geom1,c.geom2] or not ({c.geom1,c.geom2}&{self.front,self.housing}):continue
            mujoco.mj_contactForce(self.m,d,i,f);F=max(0.,f[0]);force+=F;weighted+=F*c.pos
            jp=np.zeros((3,self.m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jac(self.m,d,jp,jr,c.pos,self.bid)
            js=np.zeros_like(jp);mujoco.mj_jac(self.m,d,js,jr,c.pos,self.support)
            vel=(jp-js)@d.qvel;n=c.frame.reshape(3,3)[0]
            slip+=F*np.linalg.norm(vel-n*np.dot(n,vel))
        if force>1e-6:
            p=weighted/force;loc=R.T@(p-o);loc[2]=.0335
            return force,loc,p,slip/force
        return 0.,None,None,0.

    def run(self,stage='S2',condition='eit',offset=.03,angle=5,seed=100,bias=0.,coarse=0.,delay=0.,noise=0.,rate=20.,target=3.,shift=False,save=False,event_audit_only=False):
        m=self.m;d=mujoco.MjData(m);d.qpos[:]=self.q0
        # Tip starts 1 mm below the nominal sensor face; x offset is in sensor coordinates.
        m.body_pos[self.support]=P0+R0@np.array([offset,0,0])+np.array([0,0,-.009])
        mujoco.mj_forward(m,d)
        rng=np.random.default_rng(seed);pars=session(seed+500,shift);base=self.eit.solve(**{k:pars[k] for k in ['sigma','z']})
        latched=False;latch_t=np.nan;rhat=np.array([0.,0.,.0335]);true_latch=None;anchor=None;latch_world=None
        on=0;queue=[];last_feat=None;fhat=0.;lasttrue=None;nextscan=0.;anchor_offset=0.;signalF=0.
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
        traces=[];cumslip=0.;saturated=0;guard=False;false_latch=False;nominal=3.;horizon=7.
        for step in range(round(horizon/self.dt)):
            t=step*self.dt;mujoco.mj_forward(m,d)
            force,loc,world,slip=self.contact(d);cumslip+=slip*self.dt
            R=d.xmat[self.bid].reshape(3,3);o=d.xpos[self.bid];p=d.site_xpos[self.sid]
            # First-order material response (80 ms), with 6% unloading hysteresis.
            signalF+=(force-signalF)*min(1,self.dt/.08)
            if t+1e-9>=nextscan and (not latched or stage=='S4'):
                active=loc is not None and np.linalg.norm(loc[:2])<=.045
                live_pars=pars.copy()
                if force<signalF-.02:live_pars['gain']*=1.06
                feat=measure(self.eit,loc[:2] if active else None,signalF if active else 0.,live_pars,rng,base)
                if noise:feat+=rng.normal(0,noise,len(feat))
                pred=self.est.predict(feat[None])[0]*[.03,.03,5]
                sensed=np.linalg.norm(feat)>.008
                if active:lasttrue=loc.copy()
                # Timestamped frame and corresponding true geometry for fair oracle comparison.
                queue.append((t+delay,t,feat,pred,sensed,loc.copy() if active else None))
                nextscan+=1/rate
            while queue and queue[0][0]<=t:
                _,stamp,feat,pred,sensed,gt=queue.pop(0)
                fhat=float(pred[2]) if sensed else 0.;last_feat=feat
                on=on+1 if sensed else 0
                if not latched and on>=3:
                    # Never use truth to filter an erroneous electrical trigger.
                    # An acquisition before physical contact is a failed trial.
                    # Stop its evaluation here; do not rescue it by waiting for contact.
                    if event_audit_only:return {'false_latch':gt is None,'event_time_s':t},None
                    if gt is None:
                        false_latch=True
                        break
                    latched=True;latch_t=t;true_latch=gt.copy()
                    rhat=gt.copy() if condition in ['oracle','force_oracle','nominal','force_eit'] else np.r_[pred[:2],.0335]
                    if condition=='fixed':rhat=np.array([0.,0.,.0335])
                    rhat[0]+=bias
                    if coarse:rhat[:2]=coarse*np.round(rhat[:2]/coarse)
                    # Clamp only to the active disk; does not use true contact coordinates.
                    rr=np.linalg.norm(rhat[:2])
                    if rr>.044:rhat[:2]*=.044/rr
                    anchor=o+R@rhat;latch_world=o+R@true_latch
            if false_latch:break
            if not latched:
                r=np.array([0,0,.0335]);a=P0+np.array([0,0,-min(.014,.010*t)])
                desired_R=R0;K=np.array([1500,1500,800.]);D=np.array([70,70,60.]);kr=8.;dr=1.5
            else:
                elapsed=t-latch_t;r=rhat;K=np.array([1500.,1500.,800.]);D=np.array([70.,70.,60.]);kr=8.;dr=1.5
                a=anchor.copy();ramp=np.clip(elapsed/.4,0,1)
                a[2]-=nominal/K[2]*ramp
                theta=0.
                if stage in ['S2','S3']:
                    # 0.8 s settle, 1 s rotate, 1 s hold, 1 s return, 0.7 s hold.
                    u=np.clip((elapsed-.8)/1.,0,1);u=u*u*(3-2*u)
                    v=np.clip((elapsed-2.8)/1.,0,1);v=v*v*(3-2*v)
                    theta=np.deg2rad(angle)*(u-v)
                elif stage=='S1':
                    kr=2.;dr=.4
                    a[2]-=.008*np.clip((elapsed-.8)/1.,0,1)
                    a[2]+=.008*np.clip((elapsed-2.8)/1.,0,1)
                elif stage=='S4':
                    a[2]=anchor[2]-target/K[2]*ramp
                    if condition in ['force_eit','force_oracle']:
                        feedback=fhat if condition=='force_eit' else force
                        if elapsed>.4 and elapsed<4.5:
                            anchor_offset=np.clip(anchor_offset+np.clip(.0015*(target-feedback),-.003,.003)*self.dt,-.008,.008)
                        a[2]-=anchor_offset
                desired_R=Rotation.from_rotvec([0,theta,0]).as_matrix()@R0
                if elapsed>4.5:a[2]+=.020*np.clip((elapsed-4.5)/1.,0,1)
            pc=o+R@r;mujoco.mj_jac(m,d,jp,jr,pc,self.bid)
            fv=K*(a-pc)-D*(jp@d.qvel)
            er=Rotation.from_matrix(desired_R@R.T).as_rotvec();tv=kr*er-dr*(jr@d.qvel)
            tau=jp[:,:6].T@fv+jr[:,:6].T@tv+d.qfrc_bias[:6]-.2*d.qvel[:6]
            saturated+=int(np.any(abs(tau)>LIMITS));d.ctrl[:]=np.clip(tau,-LIMITS,LIMITS)
            rot=Rotation.from_matrix(R@R0.T).as_rotvec()[1]
            drift=np.linalg.norm((o+R@true_latch)-latch_world) if latched else np.nan
            migration=np.linalg.norm(loc[:2]-true_latch[:2]) if latched and loc is not None else np.nan
            if step%10==0:
                traces.append([t,force,fhat,rot,drift,migration,cumslip,latched,p[2],d.qpos[-1] if m.nq==7 else 0.,*(loc[:2] if loc is not None else [np.nan,np.nan]),*d.qpos[:6]])
            if not np.isfinite(d.qpos).all() or np.max(abs(d.qvel[:6]))>6.3 or force>25:
                guard=True;break
            mujoco.mj_step(m,d)
        tr=np.array(traces);e=tr[:,0]-latch_t
        hold=(e>=1.9)&(e<=2.7);task=(e>=.8)&(e<=2.8)
        if hold.any():
            achieved=float(np.rad2deg(tr[hold,3]).mean());contact=float(np.mean(tr[task,1]>.2));peak=float(tr[task,1].max());drift=float(np.nanmax(tr[task,4])*1000)
            slipmm=float((tr[task,6].max()-tr[task,6].min())*1000)
            ferr=float(np.mean(abs(tr[hold,1]-target)))
            success=bool(abs(achieved-angle)<=1 and contact>=.99 and peak<=10 and drift<=2 and not guard) if stage in ['S2','S3'] else bool(not guard)
        else:achieved=contact=peak=drift=slipmm=ferr=float('nan');success=False
        row=dict(stage=stage,condition=condition,offset_mm=offset*1000,angle_deg=angle,seed=seed,bias_mm=bias*1000,coarse_mm=coarse*1000,delay_ms=delay*1000,noise=noise,rate_hz=rate,target_N=target,shift=shift,
                 latched=latched,false_latch=false_latch,latch_time_s=float(latch_t),location_error_mm=float(np.linalg.norm(rhat[:2]-true_latch[:2])*1000) if latched else np.nan,
                 achieved_deg=achieved,contact_fraction=contact,peak_force_N=peak,anchor_drift_mm=drift,slip_mm=slipmm,force_mae_N=ferr,success=success,guard=guard,torque_saturated_steps=saturated,
                 rotation_abs_deg=float(np.max(abs(np.rad2deg(tr[hold,3])))) if hold.any() else np.nan,
                 contact_displacement_mm=float((P0[2]-np.mean(tr[hold,8]))*1000) if hold.any() else np.nan)
        if save:
            name=f'{stage}_{condition}_{offset*1000:g}_{angle:g}_{seed}_{bias*1000:g}_{coarse*1000:g}_{delay*1000:g}'
            np.savez_compressed(OUT/'trajectories'/f'{name}.npz',trace=tr,row=json.dumps(row),q0=self.q0,rhat=rhat,true_latch=true_latch if true_latch is not None else np.zeros(3))
        return row,tr

def pilot():
    (OUT/'trajectories').mkdir(exist_ok=True)
    ex=Experiment()
    for c in ['fixed','eit','oracle']:
        r,tr=ex.run(condition=c,save=True);print(r,flush=True)
    (OUT/'ur3e_scene.xml').write_text(ex.xml)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--pilot',action='store_true');args=p.parse_args()
    s0();pilot()
