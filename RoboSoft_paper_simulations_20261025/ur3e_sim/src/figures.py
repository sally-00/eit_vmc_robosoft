"""Figures generated only from saved simulations, FEM solutions, and specified geometry."""
import os
os.environ.setdefault('MUJOCO_GL','egl');os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle,FancyArrowPatch
import matplotlib.tri as mtri
from PIL import Image
import mujoco
from robot import make_model,ik
from eit import EIT
from experiments import P0,R0,OUT
ROOT=OUT.parent;FIG=ROOT/'report/figures';FIG.mkdir(parents=True,exist_ok=True)
COL={'fixed':'#cc573e','eit':'#168c85','oracle':'#435aa8','nominal':'#777777','force_eit':'#168c85','force_oracle':'#435aa8'}
LAB={'fixed':'Fixed centre','eit':'EIT location','oracle':'Known location','nominal':'Nominal VMC','force_eit':'EIT force','force_oracle':'True-force feedback'}
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':120,'savefig.dpi':190,'axes.titlesize':11})
def save(fig,name):
    fig.savefig(FIG/f'{name}.png',bbox_inches='tight',facecolor='white');fig.savefig(FIG/f'{name}.pdf',bbox_inches='tight',facecolor='white');plt.close(fig)
def render(q=None,offset=.03,spring=0.,zoom=False):
    m,xml=make_model(spring=spring);root=ET.fromstring(xml)
    base=root.find(".//body[@name='base']")
    ET.SubElement(base,'geom',type='capsule',size='.034',fromto='0 0 .025 0 0 .15185',rgba='.7 .73 .76 1',contype='0',conaffinity='0')
    asset=ET.SubElement(root,'asset');ET.SubElement(asset,'texture',type='skybox',builtin='flat',rgb1='.96 .97 .98',rgb2='.96 .97 .98',width='128',height='128')
    vis=root.find('visual');ET.SubElement(vis,'headlight',ambient='.6 .6 .6',diffuse='.8 .8 .8',specular='.2 .2 .2')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root).decode());d=mujoco.MjData(m)
    d.qpos[:]=ik(m,P0,R0)
    if q is not None:d.qpos[:6]=q
    m.body_pos[m.body('support').id]=P0+R0@np.array([offset,0,0])+[0,0,-.009]
    mujoco.mj_forward(m,d)
    renderer=mujoco.Renderer(m,height=650,width=900);cam=mujoco.MjvCamera()
    cam.lookat[:]=[.3,0,.24] if zoom else [.17,0,.24];cam.distance=.31 if zoom else .95;cam.azimuth=100;cam.elevation=-12
    renderer.update_scene(d,cam);im=renderer.render().copy();renderer.close();return im

def setup():
    fig,ax=plt.subplots(1,2,figsize=(11,4.7),gridspec_kw={'width_ratios':[1.4,1]})
    ax[0].imshow(render());ax[0].axis('off');ax[0].set_title('(a) UR3e and rounded support: simulated geometry')
    a=ax[1];a.add_patch(Circle((0,0),65,color='#b7bec7'));a.add_patch(Circle((0,0),45,color='#bfe3df'))
    for j in range(8):
        t=j*np.pi/4;x,y=45*np.cos(t),45*np.sin(t);a.add_patch(Circle((x,y),2.7,color='#c89428'));a.text(52*np.cos(t),52*np.sin(t),str(j+1),ha='center',va='center',fontsize=9)
    a.plot([0,30],[0,0],'k--',lw=1);a.plot(30,0,'o',color=COL['fixed']);a.text(15,5,'30 mm',ha='center',fontsize=9)
    a.annotate('',(-65,-74),(65,-74),arrowprops={'arrowstyle':'<->'});a.text(0,-83,'Housing diameter: 130 mm',ha='center')
    a.text(0,76,'Active disk: diameter 90 mm',ha='center');a.text(0,-99,'Rear face mounts to tool; total thickness 33.5 mm',ha='center',fontsize=8)
    a.set(xlim=(-80,80),ylim=(-107,87),aspect='equal');a.axis('off');a.set_title('(b) Eight electrodes at 45-degree intervals')
    save(fig,'01_setup')
    # Precise sequence diagram without invented numerical results.
    fig,axs=plt.subplots(1,4,figsize=(11,2.5))
    for a,label,theta,z in zip(axs,['Approach','Detect and latch','Rotate while supported','Return and release'],[0,0,5,0],[.6,0,0,.6]):
        t=np.deg2rad(theta);centre=np.array([-.65,1+z]);contact=np.array([0.,1.])
        if theta:centre=contact+np.array([np.cos(t)*(-.65),np.sin(t)*(-.65)])
        tang=np.array([np.cos(t),np.sin(t)]);normal=np.array([-np.sin(t),np.cos(t)])
        corners=np.array([centre-1.3*tang,centre+1.3*tang,centre+1.3*tang+.35*normal,centre-1.3*tang+.35*normal])
        a.fill(corners[:,0],corners[:,1],color='#bfe3df',edgecolor='#168c85');a.plot([0,0],[0,.92],color='#777',lw=5);a.add_patch(Circle((0,.92),.08,color='#cc573e'))
        if z==0:a.plot(0,1,'o',color='#435aa8',ms=6)
        a.set(xlim=(-2.1,1.1),ylim=(-.1,2.4),aspect='equal');a.axis('off');a.set_title(label,fontsize=10)
    save(fig,'02_sequence')

def electrical():
    e=EIT(.0028);v,sol,sigma,res=e.solve([.022,.009],3,return_state=True);b=e.solve()
    fig,ax=plt.subplots(1,3,figsize=(11,3.3))
    tri=mtri.Triangulation(e.nodes[:,0]*1000,e.nodes[:,1]*1000,e.tri)
    p=ax[0].tripcolor(tri,facecolors=sigma/.003,cmap='viridis',vmin=.65,vmax=1,shading='flat');fig.colorbar(p,ax=ax[0],label='Relative sheet conductance',shrink=.75)
    ax[0].set(title='(a) Assumed contact transduction',xlabel='x (mm)',ylabel='y (mm)',aspect='equal')
    p=ax[1].tricontourf(tri,sol[:e.n,0]*1000,levels=20,cmap='coolwarm');fig.colorbar(p,ax=ax[1],label='Potential (mV)',shrink=.75)
    ax[1].set(title='(b) Solved current-flow field',xlabel='x (mm)',ylabel='y (mm)',aspect='equal')
    ax[2].plot(b*1000,label='No contact',color='#777');ax[2].plot(v*1000,label='3 N synthetic contact',color=COL['eit']);ax[2].set(title='(c) Forty adjacent channels',xlabel='Channel index',ylabel='Boundary voltage (mV)');ax[2].legend(fontsize=8)
    fig.tight_layout();save(fig,'03_electrical')

def sensing():
    fig,ax=plt.subplots(1,3,figsize=(11,3.5))
    for name,label,col in [('test','Held-out sessions','#168c85'),('shift','Wider material variation','#cc573e')]:
        d=np.load(OUT/f'eit_{name}_predictions.npz');y=d['true'];p=d['pred'];err=np.linalg.norm(p[:,:2]-y[:,:2],axis=1)*1000
        ax[0].plot(np.sort(err),np.arange(1,len(err)+1)/len(err),label=label,color=col)
        ax[1].scatter(y[:,2]+(.05 if name=='shift' else -.05),p[:,2],s=7,alpha=.18,color=col,label=label)
        if name=='test':
            ax[2].quiver(y[::5,0]*1000,y[::5,1]*1000,(p[::5,0]-y[::5,0])*1000,(p[::5,1]-y[::5,1])*1000,angles='xy',scale_units='xy',scale=1,width=.003,color=col)
            ax[2].scatter(y[::5,0]*1000,y[::5,1]*1000,s=5,color=col)
    ax[0].set(xlabel='Position error (mm)',ylabel='Empirical cumulative fraction',title='(a) Position generalization',xlim=(0,15));ax[0].legend(fontsize=8)
    ax[1].plot([0,7],[0,7],'k--',lw=1);ax[1].set(xlabel='True force (N)',ylabel='Predicted force (N)',title='(b) Force ambiguity')
    ax[2].add_patch(Circle((0,0),45,fill=False,color='#999'));ax[2].set(xlabel='x (mm)',ylabel='y (mm)',title='(c) Held-out location errors',aspect='equal',xlim=(-48,48),ylim=(-48,48))
    fig.tight_layout();save(fig,'04_sensing')

def reduced():
    tr=np.load(OUT/'s0_traces.npz')['traces'];fig,ax=plt.subplots(1,3,figsize=(11,3.2))
    for t,name in zip(tr[:2],['fixed','oracle']):
        ax[0].plot(t[:,0],t[:,5]*1000,label=LAB[name],color=COL[name]);ax[1].plot(t[:,0],np.rad2deg(t[:,2]),color=COL[name])
    ax[0].set(xlabel='Time (s)',ylabel='Contact displacement (mm)',title='(a) Original 60 mm benchmark');ax[0].legend(fontsize=8)
    ax[1].set(xlabel='Time (s)',ylabel='Rotation (degrees)',title='(b) Unwanted rotation')
    b=np.linspace(-30,30,100);ax[2].plot(b,5*(b/1000)**2/2*1000,color=COL['eit']);ax[2].axhline(2.25,color=COL['fixed'],ls='--',label='Fixed centre at 30 mm');ax[2].set(xlabel='Attachment bias (mm)',ylabel='Excess displacement (mm)',title='(c) Analytical sensing tolerance');ax[2].legend(fontsize=8)
    fig.tight_layout();save(fig,'05_reduced')

def mechanics(df):
    s=df[df.stage=='S1'];fig,ax=plt.subplots(1,3,figsize=(11,3.5))
    allfits=[]
    model,_=make_model(spring=1000.);data=mujoco.MjData(model);tool=model.body('tool').id
    for c in ['fixed','eit','oracle']:
        sub=s[s.condition==c];g=sub.groupby('offset_mm').rotation_abs_deg.mean();ax[0].plot(g.index,g.values,'o-',color=COL[c],label=LAB[c])
        row=sub[(sub.offset_mm==30)&(sub.seed==700)].iloc[0];tr=np.load(OUT/'trial_data'/f'trial_{int(row.trial_id):04d}.npz')['trace'];el=tr[:,0]-row.latch_time_s
        mask=(el>=.8)&(el<=3.8);cmd=8*(np.clip((el-.8),0,1)-np.clip((el-2.8),0,1))
        ax[1].plot(cmd[mask],tr[mask,1],color=COL[c],label=LAB[c])
        # Compare matched measured force interval, not identical command progress.
        for _,r in sub[sub.offset_mm==30].iterrows():
            t=np.load(OUT/'trial_data'/f'trial_{int(r.trial_id):04d}.npz')['trace'];e=t[:,0]-r.latch_time_s
            cd=8*np.clip(e-.8,0,1);use=(e>=.8)&(e<=1.8)&(t[:,1]>=3.5)&(t[:,1]<=4.8)
            slope=np.polyfit(t[use,1],cd[use],1)[0] if use.sum()>5 else np.nan
            first=np.flatnonzero(t[:,7]>0)[0];rtrue=np.r_[t[first,10:12],.0335];zmaterial=[]
            for sample in t[use]:
                data.qpos[:6]=sample[12:18];data.qpos[6]=sample[9];mujoco.mj_forward(model,data)
                zmaterial.append((data.xpos[tool]+data.xmat[tool].reshape(3,3)@rtrue)[2]*1000)
            relative=np.polyfit(t[use,1],cd[use]+np.array(zmaterial),1)[0]
            allfits.append(dict(trial_id=int(r.trial_id),condition=c,command_compliance_mm_per_N=slope,contact_relative_compliance_mm_per_N=relative))
    fits=pd.DataFrame(allfits);fits.to_csv(OUT/'s1_matched_force_fits.csv',index=False)
    for i,c in enumerate(['fixed','eit','oracle']):
        vals=fits[fits.condition==c].contact_relative_compliance_mm_per_N;ax[2].bar(i,vals.mean(),color=COL[c]);ax[2].plot(np.full(len(vals),i),vals,'k.',ms=3)
    ax[0].set(xlabel='Contact offset (mm)',ylabel='Peak hold rotation (degrees)',title='(a) Unwanted rotation');ax[0].legend(fontsize=8)
    ax[1].set(xlabel='Additional anchor travel (mm)',ylabel='True force (N)',title='(b) Loading and unloading at +30 mm')
    ax[2].set(xticks=range(3),xticklabels=['Fixed','EIT','Known'],ylabel='Contact-relative compliance (mm/N)',title='(c) Matched force: 3.5-4.8 N')
    fig.tight_layout();save(fig,'06_mechanics')

def task(df):
    s=df[df.stage=='S2'];fig,ax=plt.subplots(1,3,figsize=(11,3.5))
    for c in ['fixed','eit','oracle']:
        sub=s[s.condition==c];g=sub.groupby('offset_mm').success.mean();ax[0].plot(g.index,g.values*100,'o-',color=COL[c],label=LAB[c])
        er=abs(sub.achieved_deg-sub.angle_deg);sub=sub.assign(error=er);g=sub.groupby('offset_mm').error.mean();ax[1].plot(g.index,g.values,'o-',color=COL[c])
        g=sub.groupby('offset_mm').peak_force_N.mean();ax[2].plot(g.index,g.values,'o-',color=COL[c])
    ax[0].set(xlabel='Contact offset (mm)',ylabel='Success (%)',ylim=(-5,105),title='(a) Composite task success');ax[0].legend(fontsize=8)
    ax[1].axhline(1,color='#777',ls='--',lw=1);ax[1].set(xlabel='Contact offset (mm)',ylabel='Mean absolute orientation error (deg)',title='(b) Achieving the requested rotation')
    ax[2].set(xlabel='Contact offset (mm)',ylabel='Mean peak force (N)',title='(c) Independent contact-force check')
    fig.tight_layout();save(fig,'07_main_results')
    fig,ax=plt.subplots(1,3,figsize=(11,3.3))
    for c in ['fixed','eit','oracle']:
        row=s[(s.condition==c)&(s.offset_mm==30)&(s.angle_deg==5)&(s.seed==710)].iloc[0]
        tr=np.load(OUT/'trial_data'/f'trial_{int(row.trial_id):04d}.npz')['trace'];t=tr[:,0]-row.latch_time_s
        mask=(t>=0)&(t<=4.3);tr=tr[mask];t=t[mask]
        ax[0].plot(t,np.rad2deg(tr[:,3]),color=COL[c],label=LAB[c]);ax[1].plot(t,tr[:,1],color=COL[c]);ax[2].plot(t,tr[:,4]*1000,color=COL[c])
    for a in ax:a.set_xlim(0,4.3);a.axvspan(1.9,2.7,color='#eeeeee',zorder=-10);a.set_xlabel('Time since latch (s)')
    ax[0].set(ylabel='Achieved rotation (deg)',title='(a) +30 mm contact, +5-degree goal');ax[0].legend(fontsize=8)
    ax[1].set(ylabel='True normal force (N)',title='(b) Contact remains finite')
    ax[2].set(ylabel='Latched material-point drift (mm)',title='(c) Drift is not the main failure')
    fig.tight_layout();save(fig,'08_task_traces')

def robustness(df):
    s=df[df.stage=='S3'];fig,ax=plt.subplots(2,3,figsize=(11,6.4))
    bias=s[(s.condition=='oracle')&s.mu.isna()]
    for a,ls in [(-5,'--'),(5,'-')]:
        g=bias[bias.angle_deg==a].groupby('bias_mm').success.mean();ax[0,0].plot(g.index,g.values*100,'o'+ls,label=f'{a:+d} degrees')
    ax[0,0].set(xlabel='Added signed location bias (mm)',ylabel='Success (%)',title='(a) Isolated geometric error',ylim=(-5,105));ax[0,0].legend(fontsize=8)
    eit=s[(s.condition=='eit')&s.mu.isna()&(~s['shift'])]
    baseline=eit[(eit.coarse_mm==0)&(eit.delay_ms==0)&(eit.rate_hz==20)&(eit.noise==0)]
    for a,col,label in [(ax[0,1],'coarse_mm','Spatial bin width (mm)'),(ax[0,2],'delay_ms','Additional frame delay (ms)'),(ax[1,0],'rate_hz','Acquisition rate (Hz)'),(ax[1,1],'noise','Added normalized voltage noise')]:
        if col=='coarse_mm':sub=pd.concat([baseline,eit[eit.coarse_mm>0]])
        elif col=='delay_ms':sub=eit[(eit.coarse_mm==0)&(eit.rate_hz==20)&(eit.noise==0)]
        elif col=='rate_hz':sub=eit[(eit.coarse_mm==0)&(eit.delay_ms==0)&(eit.noise==0)]
        else:sub=eit[(eit.coarse_mm==0)&(eit.delay_ms==0)&(eit.rate_hz==20)]
        g=sub.groupby(col).success.mean();a.plot(g.index,g.values*100,'o-',color=COL['eit']);a.set(xlabel=label,ylabel='Success (%)',ylim=(-5,105))
    stress=s[s.mu.notna()];labels=[]
    for i,((mu,soft),g) in enumerate(stress.groupby(['mu','soft'])):
        labels.append(f'{mu:g} / {soft*1000:g}')
        for j,c in enumerate(['fixed','eit','oracle']):ax[1,2].bar(i+(j-1)*.23,100*g[g.condition==c].success.mean(),width=.22,color=COL[c],label=LAB[c] if i==0 else None)
    ax[1,2].set(xticks=range(len(labels)),xticklabels=labels,xlabel='Friction / contact time constant (ms)',ylabel='Success (%)',ylim=(0,105));ax[1,2].legend(fontsize=7)
    fig.tight_layout();save(fig,'09_robustness')

def force(df):
    s=df[df.stage=='S4'];fig,ax=plt.subplots(1,3,figsize=(11,3.4))
    for a,k in zip(ax[:2],[500,1500]):
        for c in ['nominal','force_eit','force_oracle']:
            g=s[(s.spring==k)&(s.condition==c)].groupby('target_N').force_mae_N.mean();a.plot(g.index,g.values,'o-',label=LAB[c],color=COL[c])
        a.set(xlabel='Force target (N)',ylabel='True force MAE (N)',title=f'Support stiffness {k} N/m')
    ax[0].legend(fontsize=8)
    for c in ['nominal','force_eit','force_oracle']:
        r=s[(s.spring==500)&(s.condition==c)&(s.target_N==4)&(s.seed==740)].iloc[0];tr=np.load(OUT/'trial_data'/f'trial_{int(r.trial_id):04d}.npz')['trace']
        ax[2].plot(tr[:,0]-r.latch_time_s,tr[:,1],color=COL[c])
    ax[2].axhline(4,color='#777',ls='--');ax[2].set(xlim=(0,4.5),xlabel='Time since latch (s)',ylabel='True force (N)',title='4 N target; 500 N/m support')
    fig.tight_layout();save(fig,'10_force')

def extension():
    primary=pd.read_csv(OUT/'analyzed_metrics.csv');ex=pd.read_csv(OUT/'analyzed_exploratory.csv')
    fig,ax=plt.subplots(1,3,figsize=(11,3.4))
    for c in ['fixed','eit','oracle']:
        sub=pd.concat([primary[(primary.stage=='S2')&primary.offset_mm.isin([-30,0,30])],ex[ex.stage=='S2']]);sub=sub[sub.condition==c].assign(magnitude=lambda d:abs(d.angle_deg))
        g=sub.groupby('magnitude').success.mean();ax[0].plot(g.index,g.values*100,'o-',color=COL[c],label=LAB[c])
    ax[0].set(xlabel='Rotation magnitude (degrees)',ylabel='Success (%)',ylim=(-5,105),title='(a) Harder task, unchanged controller');ax[0].legend(fontsize=8)
    sub=pd.concat([primary[(primary.stage=='S3')&(primary.condition=='oracle')&primary.mu.isna()],ex[ex.stage=='S3']])
    for a,ls in [(-5,'--'),(5,'-')]:
        g=sub[sub.angle_deg==a].groupby('bias_mm').success.mean();ax[1].plot(g.index,g.values*100,'o'+ls,label=f'{a:+d} degrees')
    ax[1].axvspan(-50,-21,color='#eeeeee',zorder=-1);ax[1].set(xlabel='Added signed bias (mm)',ylabel='Success (%)',ylim=(-5,105),title='(b) Extended bias boundary');ax[1].legend(fontsize=8)
    s=primary[(primary.stage=='S3')&(primary.condition=='eit')&(primary.delay_ms==0)&(primary.coarse_mm==0)&(primary.noise==0)&primary.mu.isna()&(~primary['shift'])]
    g=s.groupby('rate_hz').pre_latch_peak_N.max();ax[2].plot(g.index,g.values,'o-',color='#cc573e');ax[2].set(xlabel='Acquisition rate (Hz)',ylabel='Peak force before latch (N)',title='(c) Slow sensing has a cost')
    fig.tight_layout();save(fig,'11_followup')

def snapshots():
    df=pd.read_csv(OUT/'exploratory_metrics.csv');r=df[(df.stage=='S2')&(df.condition=='eit')&(df.offset_mm==30)&(df.angle_deg==10)&(df.seed==800)].iloc[0]
    tr=np.load(OUT/'trial_data'/f'trial_{int(r.trial_id):04d}.npz')['trace'];fig,ax=plt.subplots(1,3,figsize=(11,3))
    for a,t,label in zip(ax,[.7,2.6,4.3],['Latched support','10-degree reorientation','Return']):
        i=np.argmin(abs(tr[:,0]-r.latch_time_s-t));a.imshow(render(q=tr[i,12:18],offset=.03,zoom=True));a.axis('off');a.set_title(f'{label}: {np.rad2deg(tr[i,3]):.1f} deg',fontsize=10)
    save(fig,'12_motion_frames')

def main():
    setup();electrical();sensing();reduced()
    df=pd.read_csv(OUT/'trial_metrics.csv');mechanics(df);task(df);robustness(df);force(df)
    if (OUT/'analyzed_exploratory.csv').exists():extension();snapshots()
if __name__=='__main__':main()
