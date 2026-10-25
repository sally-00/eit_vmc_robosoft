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
        edge1=p[:,1]-p[:,0]; edge2=p[:,2]-p[:,0]
        area=(edge1[:,0]*edge2[:,1]-edge1[:,1]*edge2[:,0])/2
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
