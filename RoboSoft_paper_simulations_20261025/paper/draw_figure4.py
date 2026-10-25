"""Figure 4: task mechanisms paired with renders of the evaluated controllers."""
from pathlib import Path
import os, sys, json
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arc, FancyBboxPatch, FancyArrowPatch
from matplotlib.path import Path as PlotPath

ROOT=Path(__file__).resolve().parent
SIM=ROOT.parent/'joystick_ball_sim'
sys.path.insert(0,str(SIM/'src'))
from figures import render

from figure_style import VIRTUAL as V, CONTACT as E, PHYSICAL as P, INK
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42,'svg.fonttype':'none'})

def line(ax,pts,color=V,**kw):
    a=np.array(pts);ax.plot(a[:,0],a[:,1],color=color,lw=1.25,**kw)

def arrow(ax,a,b,color=V,**kw):
    ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',lw=1.15,color=color,**kw))

def spring(ax,a,b):
    a=np.array(a);b=np.array(b);v=b-a;n=np.array([-v[1],v[0]])/np.linalg.norm(v)
    pts=[a,a+.15*v]
    for i,t in enumerate(np.linspace(.2,.8,9)):pts.append(a+t*v+.14*n*(-1)**i)
    pts.extend([a+.85*v,b]);line(ax,pts)

def damper(ax,a,b):
    a=np.array(a);b=np.array(b);v=b-a;n=np.array([-v[1],v[0]])/np.linalg.norm(v)
    def p(t,w=0):return a+t*v+w*n
    line(ax,[a,p(.52)]);line(ax,[p(.52,-.22),p(.52,.22)])
    line(ax,[p(.35,-.30),p(.72,-.30),p(.72,.30),p(.35,.30)])
    line(ax,[p(.72),b])

def pad(ax,x=6.8,y=9):
    ax.add_patch(Rectangle((x-1.8,y),3.6,.42,fc='#e8ecee',ec=P,lw=1))
    ax.add_patch(Rectangle((x-1.65,y),3.3,.10,fc='#b8dcdc',ec=P,lw=.65))
    ax.plot(x,y,'o',ms=4.5,color=E,zorder=5)
    ax.add_patch(Rectangle((x-.35,y+.42),.7,.55,fc='#e8ecee',ec=P,lw=.8))
    ax.text(x-2,y+.25,'Palm',color=P,fontsize=8,ha='right',va='center')
    ax.text(x+.28,y-.38,r'$p_c$',color=E,fontsize=9)
    line(ax,[(x,y),(3.3,y),(3.3,7.8)],E)
    arrow(ax,(9,8.85),(9,7.9),P)
    ax.text(9.25,8.35,r'$f_n$',fontsize=9,color=P)

def coupling(ax,x,top,bottom):
    spring(ax,(x-.55,bottom+.2),(x-.55,top-.2))
    damper(ax,(x+.55,bottom+.2),(x+.55,top-.2))
    line(ax,[(x-.55,top-.2),(x+.55,top-.2)])
    line(ax,[(x,top-.2),(x,top)])
    line(ax,[(x-.55,bottom+.2),(x+.55,bottom+.2)])
    line(ax,[(x,bottom+.2),(x,bottom)])
    ax.text(x-1.05,(top+bottom)/2,r'$K_c$',ha='right',va='center',color=V)
    ax.text(x+1.05,(top+bottom)/2,r'$D_c$',va='center',color=V)

def base(ax,title):
    ax.set(xlim=(0,10),ylim=(0,10));ax.axis('off')
    ax.set_title(title,loc='left',fontsize=9,fontweight='bold',pad=6)

def joystick(ax):
    base(ax,'(a) Joystick: arc-guided spring–damper')
    pad(ax)
    # The real cap touches the underside of the palm; the virtual branch is
    # drawn beside it, below the same attachment, with no artificial contact gap.
    ax.add_patch(Circle((6.8,8.72),.28,fc='#dce2e5',ec=P,lw=1))
    line(ax,[(6.8,8.44),(6.1,5.25)],P)
    ax.add_patch(Rectangle((5.1,4.72),2.0,.42,fc='#e8ecee',ec=P,lw=1))
    ax.plot(6.1,5.25,'o',ms=4,color=P)
    ax.text(7.4,6.25,'Physical\njoystick',color=P,fontsize=8)
    # Actual shaft angle is measured from upright at the physical pivot.
    stick_pivot=np.array([6.1,5.25])
    shaft_tip=np.array([6.8,8.44])
    shaft_angle=np.arctan2(*(shaft_tip-stick_pivot)[::-1])
    line(ax,[stick_pivot,stick_pivot+[0,2.6]],P,ls='--',alpha=.65)
    angle_radius=2.05
    angles=np.linspace(np.pi/2,shaft_angle,40)
    angle_points=stick_pivot+angle_radius*np.column_stack([np.cos(angles),np.sin(angles)])
    codes=[PlotPath.MOVETO]+[PlotPath.LINETO]*(len(angle_points)-1)
    ax.add_patch(FancyArrowPatch(path=PlotPath(angle_points,codes),arrowstyle='->',
                                mutation_scale=6.5,color=P,lw=1.05))
    ax.text(5.88,7.35,r'$\theta$',color=P,fontsize=9,ha='right')
    coupling(ax,3.3,7.8,5.5)
    ax.text(.25,8.15,'Virtual\ncoupling',color=V,fontsize=8)
    # A geometric guide for the prescribed output, not a dynamic virtual lever.
    pivot=np.array([2.4,2.35]);rad=2.05;theta=np.deg2rad(64)
    h=pivot+rad*np.array([np.cos(theta),np.sin(theta)])
    ax.add_patch(Arc(pivot,2*rad,2*rad,theta1=50,theta2=128,color=V,lw=1.4,ls='--'))
    line(ax,[pivot,h],ls=':');ax.plot(*pivot,'o',ms=3,color=V);ax.plot(*h,'o',ms=4,color=V)
    arrow(ax,(1.6,4.25),(2,4.38))
    ax.text(.15,3.3,'Prescribed\ncap arc',color=V,fontsize=8)
    ax.text(2.85,3.0,r'$L$',color=V,fontsize=9)
    ax.text(3.8,4.25,r'$h(\theta_d)$',color=V,fontsize=9)
    line(ax,[h,(3.3,h[1]),(3.3,5.5)])
    ax.text(4.65,3.1,r'$\theta_d(t): +12^\circ\rightarrow-12^\circ$',fontsize=8)
    ax.text(.25,1.4,'Normal preload: '+r'$K_n,D_n$'+'; orientation follows '+r'$\theta_d$',fontsize=7.5,color=INK)
    ax.text(.25,.5,'EIT: define VMC attachment at contact',fontsize=8,color=E)

def ball(ax):
    base(ax,'(b) Ball rolling: virtual carriage')
    # Physical scene: current ball under the palm and a separate target ball.
    x, y, radius = 4.2, 8.0, .85
    face = y+radius
    ax.add_patch(Rectangle((x-1.65,face),3.3,.38,fc='#e8ecee',ec=P,lw=1))
    ax.add_patch(Rectangle((x-1.5,face),3.0,.09,fc='#b8dcdc',ec=P,lw=.65))
    ax.add_patch(Rectangle((x-.3,face+.38),.6,.45,fc='#e8ecee',ec=P,lw=.8))
    ax.text(x+.65,face+.6,'Palm',color=P,fontsize=8,ha='left',va='center')
    ax.add_patch(Circle((x,y),radius,fc='#f7deb0',ec=P,lw=1.2))
    ax.text(x,y-.03,'Ball',ha='center',va='center',color=P,fontsize=8)
    ax.plot(x,face,'o',ms=4.5,color=E,zorder=5)
    ax.text(x+.26,face-.32,r'$p_c$',color=E,fontsize=9)
    ax.add_patch(Arc((x,y),1.12,1.12,theta1=30,theta2=150,color=P,lw=1))
    arrow(ax,(3.73,8.29),(3.67,8.14),P)
    goal_x=8.0
    ax.add_patch(Circle((goal_x,y),radius,fc='none',ec=V,lw=1.3,ls=(0,(4,3))))
    ax.text(goal_x,y,'Desired\nball',ha='center',va='center',color=V,fontsize=7.5)
    ax.text(goal_x,9.15,r'Goal $s_g$ (fixed)',ha='center',color=V,fontsize=8)
    line(ax,[(2.8,y-radius),(9.25,y-radius)],P)
    ax.text(9.25,6.83,'Table',ha='right',color=P,fontsize=7.2)
    arrow(ax,(5.25,7.65),(6.95,7.65),P)
    ax.text(6.1,7.38,'Roll',ha='center',color=P,fontsize=7.1)
    arrow(ax,(6.2,8.75),(6.2,8.12),P)
    ax.text(6.4,8.42,r'$f_n$',color=P,fontsize=9)
    # Attachment at the actual palm contact; virtual components sit below it.
    line(ax,[(x,face),(1.45,face),(1.45,7.4)],E)
    coupling(ax,1.45,7.4,5.4)
    ax.text(.1,9.05,'Virtual\ncoupling',color=V,fontsize=7.5)
    # Carriage and a goal spring whose endpoint shares the target coordinate.
    ax.add_patch(Rectangle((4.6,5.3),1.2,.85,fc='#fcf1de',ec=V,lw=1.2))
    ax.text(5.2,5.725,r'$M_v$',ha='center',va='center',color=V,fontsize=9)
    ax.text(5.2,6.42,r'Carriage $s$',ha='center',color=V,fontsize=7.5)
    spring(ax,(5.8,5.725),(goal_x,5.725))
    ax.plot(goal_x,5.725,'s',ms=4,color=V)
    ax.text(6.9,6.15,r'$k_g$ (bounded)',ha='center',color=V,fontsize=7.2)
    # This dotted guide identifies a shared position reference, not a force path.
    ax.plot([goal_x,goal_x],[7.08,5.93],color=V,lw=.85,ls=':')
    ax.text(8.25,6.4,r'$s_g$',color=V,fontsize=8)
    # Carriage damping to a stationary reference.
    line(ax,[(2.8,5.48),(2.8,5.98)],P)
    for yy in [5.5,5.7,5.9]:line(ax,[(2.8,yy),(2.6,yy-.12)],P)
    damper(ax,(2.8,5.725),(4.6,5.725))
    ax.text(3.7,5.08,r'$d_v$',ha='center',color=V,fontsize=8)
    # Solid mechanical connections transmit motion and force in both directions.
    line(ax,[(5.2,5.3),(5.2,4.11)])
    ax.text(5.48,4.65,r'$s$',color=V,fontsize=8)
    ax.add_patch(FancyBboxPatch((4.0,3.15),2.4,.9,boxstyle='round,pad=.06',fc='white',ec=V,lw=1.1))
    ax.text(5.2,3.6,'2:1\ntransmission',ha='center',va='center',color=V,fontsize=7.5)
    line(ax,[(4.,3.6),(1.45,3.6),(1.45,5.4)])
    ax.text(2.7,3.87,r'$h(s)$',ha='center',color=V,fontsize=9)
    ax.text(.25,2.25,'Normal preload: '+r'$K_n,D_n$'+'; orientation: horizontal',fontsize=7.2)
    ax.text(.25,1.35,'EIT: update attachment + correct progress',fontsize=7.5,color=E)
    ax.text(.25,.55,'Rebase reference to preserve spring extension',fontsize=7,color=E)

def main():
    preview,ax=plt.subplots(figsize=(6.4,5.8))
    ball(ax)
    ax.set_title('Ball rolling: virtual carriage',loc='left',fontsize=12,fontweight='bold',pad=10)
    preview.subplots_adjust(left=.035,right=.98,top=.92,bottom=.035)
    for ext in ['pdf','png','svg']:
        preview.savefig(ROOT/'figures'/f'ball_roll_mechanism.{ext}',dpi=220,bbox_inches='tight',facecolor='white')
    plt.close(preview)
    fig=plt.figure(figsize=(7.1,5.9))
    for fun,rect in [(joystick,[.025,.46,.455,.49]),(ball,[.52,.46,.455,.49])]:fun(fig.add_axes(rect))
    df=pd.read_csv(SIM/'results/analyzed_metrics.csv');provenance={}
    for task,rect,label in [('joystick',[.025,.13,.455,.29],'(c) Joystick simulation'),('ball',[.52,.13,.455,.29],'(d) Ball simulation')]:
        group='J2' if task=='joystick' else 'B1'
        offset=.025 if task=='joystick' else 0.
        seed=300 if task=='joystick' else 400
        row=df[(df.group==group)&(df.attachment=='eit')&(df.switching=='event')&
               np.isclose(df.offset,offset)&np.isclose(df.gap,.006)&(df.seed==seed)].iloc[0]
        path=SIM/'results/trials'/f'trial_{int(row.trial_id):04d}.npz'
        with np.load(path) as d:
            i=int(np.argmin(abs(d['trace'][:,0]-(row.first_engagement_s+2.5))))
            im=render(d,i,zoom=True)
            provenance[task]=dict(trial=int(row.trial_id),frame=i,time=float(d['trace'][i,0]),mechanism=row.mechanism)
        ax=fig.add_axes(rect);ax.imshow(im);ax.axis('off')
        ax.set_title(label,loc='left',fontsize=8.5,fontweight='bold',pad=3)
    fig.text(.5,.085,'Amber: virtual components     Blue dots / links: selected attachment     Lower panels: saved MuJoCo states',ha='center',fontsize=7.3,color=INK)
    fig.text(.5,.035,r'$f_c=P_t[K_c(h-p_c)+D_c(v_{\mathrm{ref}}-J_c\dot q)],\qquad \tau_{task}=J_c^T(f_c+f_n)+J_\omega^Tm_R$',ha='center',fontsize=8.5)
    for ext in ['pdf','png','svg']:fig.savefig(ROOT/'figures'/f'task_setup_mechanisms.{ext}',dpi=260,bbox_inches='tight',facecolor='white')
    plt.close(fig)
    (ROOT/'evidence/task_simulations/figure4_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps(provenance))

if __name__=='__main__':main()
