"""Setup renders, exact mechanism diagrams, and plots of saved evaluation data."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch
import mujoco
from scene import ROOT

FIG=ROOT/'report/figures';FIG.mkdir(parents=True,exist_ok=True)
OUT=ROOT/'results'
COL={'timed_centre':'#a2a6aa','timed_eit':'#c87943','event_centre':'#bd4d4e',
     'event_eit':'#168c85','event_oracle':'#435aa8','event_eit_point':'#8c5aa7'}
LAB={'timed_centre':'Timed / centre','timed_eit':'Timed / EIT',
     'event_centre':'Touch / centre','event_eit':'Touch / EIT',
     'event_oracle':'Touch / known location','event_eit_point':'Touch / EIT / point'}
plt.rcParams.update({'font.size':9,'axes.titlesize':10,'axes.spines.top':False,
                     'axes.spines.right':False,'figure.dpi':120,'savefig.dpi':190})


def save(fig,name):
    for ext in ['png','pdf']:
        fig.savefig(FIG/f'{name}.{ext}',bbox_inches='tight',facecolor='white')
    plt.close(fig)


def load(trial_id):
    return np.load(OUT/'trials'/f'trial_{int(trial_id):04d}.npz')


def policy(df):
    p=df.switching+'_'+df.attachment
    return p.where(df.mechanism!='point',p+'_point')


def render(saved,index,zoom=False):
    m=mujoco.MjModel.from_xml_string(str(saved['xml']));d=mujoco.MjData(m)
    d.qpos[:]=saved['qpos'][index]
    if m.nmocap:
        d.mocap_pos[:]=saved['mocap'][index]
    mujoco.mj_forward(m,d)
    cam=mujoco.MjvCamera();cam.azimuth=105;cam.elevation=-12
    if zoom:
        cam.lookat[:]=[d.xpos[m.body('object').id][0],0,.285];cam.distance=.32
        if m.nq>7:
            cam.lookat[2]=.265;cam.azimuth=90;cam.elevation=0
    else:
        cam.lookat[:]=[.20,0,.26];cam.distance=1.0;cam.elevation=-16
    renderer=mujoco.Renderer(m,height=650,width=900)
    renderer.update_scene(d,cam);im=renderer.render().copy();renderer.close()
    return im


def scenes(df):
    selections={
        'setup_joystick':('J1',dict(switching='event',attachment='eit',gap=.006,offset=.025,seed=300)),
        'setup_ball':('B1',dict(switching='event',attachment='eit',gap=.006,offset=0.,seed=400))}
    for name,(group,criteria) in selections.items():
        sub=df[df.group==group]
        for k,v in criteria.items():sub=sub[sub[k]==v]
        row=sub.iloc[0];d=load(row.trial_id);tr=d['trace']
        engaged=row.first_engagement_s
        i=np.argmin(abs(tr[:,0]-(engaged+2.5)))
        fig,ax=plt.subplots(1,2,figsize=(10.5,3.8))
        ax[0].imshow(render(d,i));ax[0].set_title('UR3e and physical task fixture')
        ax[1].imshow(render(d,i,True));ax[1].set_title('Actual simulated contact, enlarged')
        for a in ax:a.axis('off')
        fig.suptitle(f"Trial {int(row.trial_id)}; t={tr[i,0]:.2f} s; normal force={tr[i,1]:.2f} N",fontsize=10)
        fig.tight_layout();save(fig,name)
    # Saved states rather than a choreographed visualization.
    row=df[(df.group=='J1')&(df.switching=='event')&(df.attachment=='eit')&
           (df.gap==.006)&(df.offset==.025)&(df.seed==300)].iloc[0]
    d=load(row.trial_id);tr=d['trace'];a=row.first_engagement_s
    snapshot(d,[.0,a+3.8,a+8.5],['Approach','Positive joystick input','Negative joystick input'],
             'motion_joystick',angle=True)
    row=df[(df.group=='B3')&(df.switching=='event')&(df.attachment=='eit')&
           (df.mechanism=='mechanism')&(df.offset==.01)&(df.seed==600)].iloc[0]
    d=load(row.trial_id);events=json.loads(str(d['events']))
    times=[0.]+[e['t']+.2 for e in events if e['to'] in ['lift','reposition']][:2]
    engagements=[e['t'] for e in events if e['to']=='engage']
    times.append(engagements[1]+1.5 if len(engagements)>1 else row.duration_s)
    snapshot(d,times,['Approach','Planned lift','Reposition','Next rolling\nstroke'],
             'motion_multistroke')
    row=df[(df.group=='B4')&(df.switching=='event')&(df.attachment=='eit')&
           (df.mechanism=='mechanism')&(df.offset==.01)&(df.seed==700)].iloc[0]
    snapshot(load(row.trial_id),[2.3,2.9,3.6,4.5],
             ['Before\ninterruption','Support\nwithdrawn','Touch-based\nrecovery','Support\nrestored'],
             'motion_interruption')
    fig,ax=plt.subplots(1,3,figsize=(11,3.2))
    for a,r in zip(ax,[.02,.025,.03]):
        row=df[(df.group=='B5')&(df.attachment=='eit')&(df.radius==r)&(df.mu==.4)&(df.seed==800)].iloc[0]
        d=load(row.trial_id);i=np.argmin(abs(d['trace'][:,0]-2.0))
        a.imshow(render(d,i,True));a.set_title(f'Ball diameter {r*2000:.0f} mm');a.axis('off')
    fig.tight_layout();save(fig,'ball_sizes')


def snapshot(d,times,titles,name,angle=False):
    tr=d['trace'];fig,axes=plt.subplots(1,len(times),figsize=(11,3.1))
    for k,(a,t,title) in enumerate(zip(axes,times,titles)):
        i=np.argmin(abs(tr[:,0]-t));a.imshow(render(d,i,k>0));a.axis('off');a.set_title(title)
        extra=f'; stick={np.degrees(tr[i,3]):+.1f} deg' if angle else f'; ball x={tr[i,4]*1000:.1f} mm'
        a.text(.5,-.035,f't={tr[i,0]:.2f} s; F={tr[i,1]:.1f} N\n'+extra.lstrip('; '),
               transform=a.transAxes,ha='center',fontsize=7)
    fig.tight_layout();save(fig,name)


def main_results(df,task,groups,name):
    sub=df[df.group.isin(groups)].copy();sub['policy']=policy(sub)
    fig,ax=plt.subplots(1,3,figsize=(11,3.5))
    for p in LAB:
        s=sub[sub.policy==p]
        if s.empty:continue
        g=s.groupby('gap').success.mean();ax[0].plot(g.index*1000,g.values*100,'o-',color=COL[p],label=LAB[p])
        s=s[s.gap==.006];g=s.groupby('offset').success.mean()
        ax[1].plot(g.index*1000,g.values*100,'o-',color=COL[p])
        g=s.groupby('offset').peak_force_N.mean();ax[2].plot(g.index*1000,g.values,'o-',color=COL[p])
    ax[0].set(xlabel='Initial contact gap (mm)',ylabel='Episode pass (%)',ylim=(-4,104),title='(a) All landing offsets')
    ax[1].set(xlabel='Initial landing offset (mm)',ylabel='Episode pass (%)',ylim=(-4,104),title='(b) Nominal 6 mm gap')
    ax[2].set(xlabel='Initial landing offset (mm)',ylabel='Mean peak normal force (N)',title='(c) Nominal 6 mm gap')
    ax[0].legend(fontsize=6.4,loc='lower left');fig.tight_layout();save(fig,name)


def joystick_trace(df):
    fig,ax=plt.subplots(1,3,figsize=(11,3.3))
    for attachment in ['centre','eit','oracle']:
        row=df[(df.group=='J1')&(df.switching=='event')&(df.attachment==attachment)&
               (df.gap==.006)&(df.offset==.025)&(df.seed==300)].iloc[0]
        d=load(row.trial_id);tr=d['trace'];c=COL['event_'+attachment]
        ax[0].plot(tr[:,0],np.degrees(tr[:,3]),color=c,label=LAB['event_'+attachment])
        ax[1].plot(tr[:,0],tr[:,1],color=c)
        ax[2].plot(tr[:,0],tr[:,9],color=c)
        if attachment=='eit':
            a=row.first_engagement_s
            ax[0].plot([a+.55,a+4.5,a+4.5,a+9.1],[12,12,-12,-12],'k--',lw=1,label='Requested input')
    ax[0].set(xlabel='Time (s)',ylabel='True joystick angle (deg)',title='(a) +25 mm landing');ax[0].legend(fontsize=7)
    ax[1].set(xlabel='Time (s)',ylabel='True normal force (N)',title='(b) Actual contact load')
    ax[2].set(xlabel='Time (s)',ylabel='Tactile contact validity',yticks=[0,1],title='(c) Validity and losses')
    fig.tight_layout();save(fig,'joystick_traces')


def mechanism_results(df):
    fig,ax=plt.subplots(1,3,figsize=(11,3.4))
    for task,groups,a in [('joystick',['J1','J2'],ax[0]),('ball',['B1','B2'],ax[1])]:
        s=df[df.group.isin(groups)&(df.switching=='event')&(df.attachment=='eit')&(df.gap==.006)]
        for mech,color,label in [('mechanism','#168c85','Virtual mechanism'),('point','#8c5aa7','Point spring')]:
            sub=s[s.mechanism==mech].copy()
            sub.loc[sub.target_error>=179,'target_error']=np.nan
            g=sub.groupby('offset').target_error.mean();a.plot(g.index*1000,g.values,'o-',color=color,label=label)
        a.set(xlabel='Landing offset (mm)',ylabel='Mean error (deg)' if task=='joystick' else 'Mean endpoint error (mm)',
              title='(a) Joystick input' if task=='joystick' else '(b) Short ball roll')
    ax[0].legend(fontsize=8)
    s=df[df.group=='D1']
    for i,task in enumerate(['joystick','ball']):
        for j,mech in enumerate(['mechanism','point']):
            sub=s[(s.task==task)&(s.mechanism==mech)]
            ax[2].bar(i+(j-.5)*.3,sub.peak_force_N.mean(),width=.28,
                      color=['#168c85','#8c5aa7'][j],label=mech if i==0 else None)
    ax[2].set(xticks=[0,1],xticklabels=['Joystick','Ball'],ylabel='Mean peak normal force (N)',title='(c) Temporary resistance')
    fig.tight_layout();save(fig,'mechanism_comparison')


def ball_trace(df):
    fig,ax=plt.subplots(1,3,figsize=(11,3.4))
    for attachment in ['centre','eit','oracle']:
        row=df[(df.group=='B1')&(df.switching=='event')&(df.attachment==attachment)&
               (df.gap==.006)&(df.offset==0)&(df.seed==400)].iloc[0]
        d=load(row.trial_id);tr=d['trace'];c=COL['event_'+attachment]
        x=(tr[:,4]-tr[0,4])*1000
        ax[0].plot(tr[:,0],x,color=c,label=LAB['event_'+attachment])
        ax[0].plot(tr[:,0],(tr[:,13]-tr[0,4])*1000,color=c,ls=':',lw=1)
        ax[1].plot(tr[:,0],x-.025*tr[:,6]*1000,color=c)
        ax[2].plot(tr[:,0],tr[:,7]*1000,color=c)
    ax[0].axhline(25,color='#555',ls='--',lw=1)
    ax[0].set(xlabel='Time (s)',ylabel='Ball progress (mm)',title='(a) True (solid), tactile estimate (dotted)');ax[0].legend(fontsize=7)
    ax[1].set(xlabel='Time (s)',ylabel=r'$\Delta x-R\int\omega_y dt$ (mm)',title='(b) Rotation checks actual rolling')
    ax[2].set(xlabel='Time (s)',ylabel='Estimated contact x on palm (mm)',title='(c) Contact migration')
    fig.tight_layout();save(fig,'ball_traces')


def multistroke(df):
    s=df[df.group=='B3'].copy();s['policy']=policy(s)
    keys=[p for p in LAB if (s.policy==p).any()]
    fig,ax=plt.subplots(1,3,figsize=(11,3.5))
    for a,col,title in zip(ax,['success','duration_s','premature_engagements'],
                            ['(a) Episode pass','(b) Completion or timeout','(c) Premature engagements']):
        vals=[s[s.policy==p][col].mean()*(100 if col=='success' else 1) for p in keys]
        a.bar(range(len(keys)),vals,color=[COL[p] for p in keys])
        a.set(xticks=range(len(keys)),xticklabels=[LAB[p].replace(' / ','\n') for p in keys],title=title)
        a.tick_params(axis='x',labelsize=6.5)
    ax[0].set_ylabel('Pass (%)');ax[1].set_ylabel('Time (s)');ax[2].set_ylabel('Mean count per episode')
    fig.tight_layout();save(fig,'multistroke_results')
    fig,ax=plt.subplots(3,1,figsize=(10.5,5.1),sharex=True)
    for switching in ['timed','event']:
        row=s[(s.switching==switching)&(s.attachment=='eit')&(s.mechanism=='mechanism')&
              (s.offset==.01)&(s.seed==600)].iloc[0]
        d=load(row.trial_id);tr=d['trace'];c=COL[switching+'_eit']
        ax[0].plot(tr[:,0],(tr[:,4]-tr[0,4])*1000,color=c,label=LAB[switching+'_eit'])
        ax[0].plot(tr[:,0],(tr[:,13]-tr[0,4])*1000,color=c,ls=':',lw=.8)
        ax[1].step(tr[:,0],tr[:,10],where='post',color=c)
        ax[2].plot(tr[:,0],tr[:,1],color=c)
    ax[0].axhline(80,color='#555',ls='--',lw=1)
    ax[0].set_ylabel('Global ball progress (mm)');ax[0].legend(fontsize=8)
    ax[1].set(ylabel='Supervisor phase',yticks=[0,1,2,3,4,5],
              yticklabels=['Approach','Engage','Interact','Brake','Lift','Reposition'])
    ax[2].set(xlabel='Time (s)',ylabel='True normal force (N)')
    fig.tight_layout();save(fig,'multistroke_traces')


def loss(df):
    s=df[df.group=='B4'].copy();s['policy']=policy(s)
    keys=[p for p in LAB if (s.policy==p).any()]
    fig,ax=plt.subplots(1,3,figsize=(11,3.5))
    for a,col,title,ylabel in zip(ax,['disconnected_motion_mm','peak_force_N','success'],
            ['(a) Motion during disconnection','(b) Load cost of recovery','(c) Eventual task outcome'],
            ['Mean horizontal travel (mm)','Mean peak normal force (N)','Episode pass (%)']):
        a.bar(range(len(keys)),[s[s.policy==p][col].mean()*(100 if col=='success' else 1) for p in keys],color=[COL[p] for p in keys])
        a.set(xticks=range(len(keys)),xticklabels=[LAB[p].replace(' / ','\n') for p in keys],title=title,ylabel=ylabel)
        a.tick_params(axis='x',labelsize=6.5)
    fig.tight_layout();save(fig,'loss_results')
    fig,ax=plt.subplots(3,1,figsize=(10.5,5.7),sharex=True)
    for switching in ['timed','event']:
        row=s[(s.switching==switching)&(s.attachment=='eit')&(s.mechanism=='mechanism')&
              (s.offset==.01)&(s.seed==700)].iloc[0]
        d=load(row.trial_id);tr=d['trace'];c=COL[switching+'_eit']
        ax[0].plot(tr[:,0],tr[:,1],color=c,label=LAB[switching+'_eit'])
        ax[1].plot(tr[:,0],(tr[:,14]-tr[0,14])*1000,color=c)
        ax[2].step(tr[:,0],tr[:,10],where='post',color=c)
    for a in ax:a.axvspan(2.5,4.25,color='#e9e9e9',zorder=-1);a.set_xlim(1.5,6)
    ax[0].set_ylabel('True force (N)');ax[0].legend(fontsize=8)
    ax[1].set_ylabel('Palm x travel (mm)')
    ax[2].set(xlabel='Time (s)',ylabel='Supervisor phase',yticks=[0,1,2,3,4,5,6],
              yticklabels=['Approach','Engage','Interact','Brake','Lift','Reposition','Recover'])
    fig.tight_layout();save(fig,'loss_traces')


def robustness(df):
    s=df[df.group=='B5'];fig,ax=plt.subplots(1,2,figsize=(10,3.5))
    for a,attachment in zip(ax,['centre','eit']):
        table=s[s.attachment==attachment].pivot_table(index='radius',columns='mu',values='success',aggfunc='mean')
        a.imshow(table.values*100,vmin=0,vmax=100,cmap='YlGnBu',aspect='auto')
        for i in range(len(table.index)):
            for j in range(len(table.columns)):
                a.text(j,i,f'{table.values[i,j]*100:.0f}%',ha='center',va='center',color='white' if table.values[i,j]>.5 else '#17344b')
        a.set(xticks=range(3),xticklabels=table.columns,yticks=range(3),yticklabels=[f'{r*2000:.0f}' for r in table.index],
              xlabel='Assumed sliding friction coefficient',ylabel='Ball diameter (mm)',title=f'{attachment.capitalize()} attachment: 40 mm roll')
    fig.tight_layout();save(fig,'size_friction')
    s=df[df.group=='S1'];fig,ax=plt.subplots(1,3,figsize=(11,3.3))
    for task,c in [('joystick','#168c85'),('ball','#435aa8')]:
        ss=s[(s.task==task)&(~s.sensor_shift.fillna(False))]
        for a,col in zip(ax,['success','first_engagement_s','pre_engagement_peak_N']):
            g=ss.groupby('rate')[col].mean();a.plot(g.index,g.values*(100 if col=='success' else 1),'o-',color=c,label=task)
    ax[0].set(xlabel='Acquisition rate (Hz)',ylabel='Episode pass (%)',ylim=(-4,104),title='(a) Sensing bandwidth');ax[0].legend()
    ax[1].set(xlabel='Acquisition rate (Hz)',ylabel='Time to engagement (s)',title='(b) Acquisition delay')
    ax[2].set(xlabel='Acquisition rate (Hz)',ylabel='Mean peak force before engagement (N)',title='(c) Cost of waiting')
    fig.tight_layout();save(fig,'sensing_results')


def mechanisms():
    fig,ax=plt.subplots(1,2,figsize=(11,4.0))
    for a in ax:a.set(xlim=(0,10),ylim=(0,7),aspect='equal');a.axis('off')
    a=ax[0];a.set_title('(a) Joystick: virtual lever',fontweight='bold')
    a.plot([2,2],[1,4.6],lw=5,color='#ce6537');a.add_patch(Circle((2,1),.18,color='#17344b'))
    a.add_patch(Circle((2,4.6),.4,color='#ce6537'))
    a.plot([2,3.2],[1,4.4],lw=3,color='#168c85',ls='--');a.add_patch(Circle((3.2,4.4),.12,color='#168c85'))
    a.annotate('',xy=(3.25,4.4),xytext=(2,4.6),arrowprops=dict(arrowstyle='->',connectionstyle='arc3,rad=-.4',color='#168c85'))
    a.text(3.6,4.7,r'$h(\theta)$ follows the cap arc',fontsize=9)
    a.add_patch(Rectangle((4.4,3.1),3.7,.5,facecolor='#dce4e9',edgecolor='#17344b'))
    a.plot([3.2,4.4],[4.4,3.35],color='#168c85',lw=2)
    a.text(4.4,3.85,r'Virtual $K_c,D_c$ coupling',fontsize=9,color='#168c85')
    a.text(4.8,2.7,r'Selected palm point $p_c$',fontsize=9)
    a.text(.7,.3,'Physical joystick pivot',fontsize=9)
    a.text(.4,6.2,r'$I_v\ddot\theta=u_g-d_v\dot\theta-G^T f_c$',fontsize=12)
    a.text(.4,5.5,'Goal torque drives the lever; contact reaction slows it.',fontsize=9)
    a=ax[1];a.set_title('(b) Ball: virtual carriage and rolling transmission',fontweight='bold')
    a.plot([.7,9.3],[1,1],color='#17344b',lw=2)
    a.add_patch(Circle((3.1,2),1,facecolor='#edba9e',edgecolor='#ce6537',lw=2))
    a.add_patch(Rectangle((1.2,3),5.4,.4,facecolor='#dce4e9',edgecolor='#17344b'))
    a.annotate('',xy=(6.3,3.9),xytext=(2.2,3.9),arrowprops=dict(arrowstyle='->',color='#168c85',lw=2))
    a.text(4.25,4.15,r'Palm motion: $2\Delta s$',ha='center',color='#168c85')
    a.annotate('',xy=(5.2,2),xytext=(3.1,2),arrowprops=dict(arrowstyle='->',color='#ce6537',lw=2))
    a.text(5.45,2,r'Ball progress: $\Delta s$',fontsize=9)
    a.text(.5,6.2,r'$h(s)=h_0+2t(s-s_0),\quad G=2t$',fontsize=12)
    a.text(.5,5.5,r'$M_v\ddot s=u_g-d_v\dot s-G^T f_c$',fontsize=12)
    a.text(.5,.3,'Rotation and slip are measured from physical sphere dynamics.',fontsize=9)
    fig.tight_layout();save(fig,'mechanisms')


def main():
    df=pd.read_csv(OUT/'analyzed_metrics.csv')
    for col in ['radius','rate','sensor_shift']:
        if col not in df:df[col]=np.nan
    scenes(df);mechanisms()
    main_results(df,'joystick',['J1','J2'],'joystick_results')
    main_results(df,'ball',['B1','B2'],'ball_results')
    joystick_trace(df);mechanism_results(df);ball_trace(df)
    multistroke(df);loss(df);robustness(df)


if __name__=='__main__':main()
