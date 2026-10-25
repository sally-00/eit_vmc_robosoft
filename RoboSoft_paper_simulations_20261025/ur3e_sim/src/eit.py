"""Eight finite-electrode, 2-D linear FEM complete-electrode model.

Units: metres, amperes, volts, sheet conductance siemens, boundary impedance ohm m.
This is a configurable synthetic sensor, NOT a calibrated TouchLoop digital twin.
"""
from pathlib import Path
import json, time
import numpy as np
from scipy.spatial import Delaunay
from scipy.sparse import coo_matrix, bmat, csc_matrix
from scipy.sparse.linalg import splu
from sklearn.ensemble import ExtraTreesRegressor
import joblib

ROOT=Path(__file__).resolve().parents[1]

class EIT:
    def __init__(self, spacing=.0035, protocol='adjacent', angle=0., boundary_n=128):
        self.radius=.045; self.protocol=protocol
        g=np.arange(-.045+spacing/2,.045,spacing)
        xx,yy=np.meshgrid(g,g); pts=np.c_[xx.ravel(),yy.ravel()]
        pts=pts[np.linalg.norm(pts,axis=1)<.045-spacing*.6]
        th=np.arange(boundary_n)*2*np.pi/boundary_n+angle
        self.nb=len(pts); self.nodes=np.r_[pts,.045*np.c_[np.cos(th),np.sin(th)]]
        self.tri=Delaunay(self.nodes).simplices
        p=self.nodes[self.tri]
        area=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0])/2
        self.area=abs(area)
        b=np.stack([p[:,1,1]-p[:,2,1],p[:,2,1]-p[:,0,1],p[:,0,1]-p[:,1,1]],1)/(2*area[:,None])
        c=np.stack([p[:,2,0]-p[:,1,0],p[:,0,0]-p[:,2,0],p[:,1,0]-p[:,0,0]],1)/(2*area[:,None])
        self.ke=self.area[:,None,None]*(b[:,:,None]*b[:,None,:]+c[:,:,None]*c[:,None,:])
        self.centres=p.mean(1); self.n=len(self.nodes)
        self.rows=np.repeat(self.tri,3,axis=1).ravel(); self.cols=np.tile(self.tri,(1,3)).ravel()
        self.edges=[]
        for j in range(boundary_n):
            mid=(th[j]+np.pi/boundary_n-angle)%(2*np.pi)
            el=int(np.round(mid/(np.pi/4)))%8
            diff=(mid-el*np.pi/4+np.pi)%(2*np.pi)-np.pi
            if abs(diff)<np.deg2rad(5.7):
                a=self.nb+j; bnd=self.nb+(j+1)%boundary_n
                self.edges.append((a,bnd,el,np.linalg.norm(self.nodes[a]-self.nodes[bnd])))
        self.drive=[(i,(i+1)%8) for i in range(8)] if protocol=='adjacent' else [(i,i+4) for i in range(4)]
        self.channels=[(d,j,(j+1)%8) for d,(a,bnd) in enumerate(self.drive) for j in range(8)
                       if protocol!='adjacent' or not ({j,(j+1)%8}&{a,bnd})]
        self.rhs=np.zeros((self.n+9,len(self.drive)))
        for i,(a,bnd) in enumerate(self.drive):
            self.rhs[self.n+a,i]=1e-4;self.rhs[self.n+bnd,i]=-1e-4

    def solve(self, pos=None, force=0., sigma=.003, z=None, gain=1., width=.005,
              sign=-1., shunt=0., return_state=False):
        z=np.full(8,.02) if z is None else np.broadcast_to(z,(8,))
        change=np.zeros(len(self.tri))
        if pos is not None:
            shape=np.exp(-np.sum((self.centres-np.array(pos))**2,1)/(2*width**2))
            change=sign*.55*gain*force/(force+2.)*shape
        conductance=sigma*np.maximum(.1,1+change)
        vals=(conductance[:,None,None]*self.ke).ravel().tolist()
        rows=self.rows.tolist();cols=self.cols.tolist()
        for a,b,e,L in self.edges:
            for i,j,v in [(a,a,2),(a,b,1),(b,a,1),(b,b,2)]:
                rows.append(i);cols.append(j);vals.append(L/(6*z[e])*v)
            for i in [a,b]:
                rows.extend([i,self.n+e]);cols.extend([self.n+e,i]);vals.extend([-L/(2*z[e])]*2)
            rows.append(self.n+e);cols.append(self.n+e);vals.append(L/z[e])
        if shunt and pos is not None:
            # Optional distributed connection to an external zero-volt electrode.
            lump=self.area*shape*shunt/3
            for k in range(3):
                rows.extend(self.tri[:,k]);cols.extend(self.tri[:,k]);vals.extend(lump)
        A=coo_matrix((vals,(rows,cols)),shape=(self.n+8,self.n+8)).tocsc()
        if shunt:
            # Grounded shunt fixes potential: no electrode zero-mean gauge constraint.
            sol=splu(A).solve(self.rhs[:-1])
        else:
            g=np.r_[np.zeros(self.n),np.ones(8)][:,None]
            A=bmat([[A,csc_matrix(g)],[csc_matrix(g.T),None]],format='csc')
            sol=splu(A).solve(self.rhs)
        U=sol[self.n:self.n+8]
        v=np.array([U[a,d]-U[b,d] for d,a,b in self.channels])
        if return_state:return v,sol,conductance,float(np.max(abs(A@sol-self.rhs[:len(sol)])))
        return v

def session(seed,shift=False):
    r=np.random.default_rng(seed)
    return dict(sigma=.003*np.exp(r.normal(0,.08 if not shift else .2)),
                z=.02*np.exp(r.normal(0,.15 if not shift else .4,8)),
                gain=np.exp(r.normal(0,.12 if not shift else .35)),
                width=r.uniform(.004,.006) if not shift else r.uniform(.003,.008))

def measure(model,pos,force,pars,rng,baseline=None,noise=.0002):
    base=model.solve(**{k:v for k,v in pars.items() if k in ['sigma','z']}) if baseline is None else baseline
    v=model.solve(pos,force,**pars)
    scale=np.sqrt(np.mean(base**2))
    # 16-bit +/-0.2 V ADC, independent channel noise + a shared offset drift.
    v=v+rng.normal(0,noise*scale,len(v))+rng.normal(0,.00005*scale)
    v=np.clip(np.round(v/(.4/65536))*(.4/65536),-.2,.2)
    return (v-base)/scale

def dataset(model,n_episodes,seed,shift=False):
    rng=np.random.default_rng(seed); X=[];Y=[];ids=[];split_sessions=[]
    for ep in range(n_episodes):
        sid=seed*100+ep//12
        pars=session(sid,shift);base=model.solve(**{k:pars[k] for k in ['sigma','z']})
        r=.037*np.sqrt(rng.random());ang=rng.uniform(-np.pi,np.pi);pos=r*np.array([np.cos(ang),np.sin(ang)])
        # All samples of a contact episode stay in the same dataset split.
        for F,unload in [(1,False),(3,False),(6,False),(3,True),(1,True)]:
            par=pars.copy();par['gain']*=1.06 if unload else 1.
            X.append(measure(model,pos,F,par,rng,base));Y.append([*pos,F]);ids.append(ep);split_sessions.append(sid)
    return np.array(X),np.array(Y),np.array(ids),np.array(split_sessions)

def run_training():
    t=time.time();out=ROOT/'results';out.mkdir(exist_ok=True)
    models={'train':EIT(.004),'validation':EIT(.0035),'test':EIT(.0028,angle=.003),'shift':EIT(.0028,angle=.008)}
    sets={}
    for name,ne,seed in [('train',480,11),('validation',80,22),('test',100,33),('shift',100,44)]:
        sets[name]=dataset(models[name],ne,seed,name=='shift')
        np.savez_compressed(out/f'eit_{name}.npz',X=sets[name][0],Y=sets[name][1],episode=sets[name][2],session=sets[name][3])
        print('EIT dataset',name,ne,'episodes',round(time.time()-t,1),'s',flush=True)
    # One development comparison using validation, never test, selects tree leaf size.
    candidates=[]
    for leaf in [1,3,6]:
        m=ExtraTreesRegressor(n_estimators=100,min_samples_leaf=leaf,max_features=1.,random_state=51,n_jobs=1)
        # Normalize target units so location and force contribute comparably to split choice.
        m.fit(sets['train'][0],sets['train'][1]/[.03,.03,5])
        pred=m.predict(sets['validation'][0])*[.03,.03,5]
        candidates.append((np.mean(np.linalg.norm(pred[:,:2]-sets['validation'][1][:,:2],axis=1)),leaf,m))
    _,leaf,m=min(candidates,key=lambda x:x[0]);joblib.dump(m,out/'eit_estimator.joblib')
    metrics={'selected_min_samples_leaf':leaf,'validation_candidates_mm':{str(l):float(e*1000) for e,l,_ in candidates},'datasets':{}}
    for name,(X,Y,ids,sids) in sets.items():
        pred=m.predict(X)*[.03,.03,5];err=np.linalg.norm(pred[:,:2]-Y[:,:2],axis=1)*1000
        np.savez_compressed(out/f'eit_{name}_predictions.npz',true=Y,pred=pred,episode=ids,session=sids)
        metrics['datasets'][name]={'episodes':len(np.unique(ids)),'sessions':len(np.unique(sids)),'location_median_mm':float(np.median(err)),
            'location_p95_mm':float(np.quantile(err,.95)),'force_mae_N':float(np.mean(abs(pred[:,2]-Y[:,2])))}
    # Numerical checks, including mesh convergence on difference measurements.
    checks={}; pos=[.02,-.012]; probes=[]
    for h in [.004,.0028,.002]:
        f=EIT(h);v,sol,c,res=f.solve(pos,3,return_state=True);b=f.solve();probes.append((v-b))
        checks[str(h)]={'nodes':f.n,'triangles':len(f.tri),'residual':res,'electrode_mean_V':float(abs(sol[f.n:f.n+8].mean(0)).max())}
    checks['relative_delta_voltage_mesh_4_to_2mm']=float(np.linalg.norm(probes[0]-probes[-1])/np.linalg.norm(probes[-1]))
    checks['relative_delta_voltage_mesh_2p8_to_2mm']=float(np.linalg.norm(probes[1]-probes[-1])/np.linalg.norm(probes[-1]))
    f=EIT(.003);v,s,c,res=f.solve(return_state=True)
    # Current drive electrode voltage differences form a symmetric transfer matrix.
    T=np.array([s[f.n+a]-s[f.n+b] for a,b in f.drive])
    checks['reciprocity_relative_error']=float(np.linalg.norm(T-T.T)/np.linalg.norm(T))
    metrics['numerical_checks']=checks;metrics['elapsed_seconds']=time.time()-t
    (out/'eit_metrics.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2),flush=True)

if __name__=='__main__':run_training()
