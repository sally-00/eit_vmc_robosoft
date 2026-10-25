"""Vector schematics of the structures actually used in the manuscript tasks."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch, Arc

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'figures'
from figure_style import VIRTUAL as V, CONTACT as E, PHYSICAL as P, INK
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42,
                     'svg.fonttype':'none'})

def line(ax,points,color=INK,**kw):
    a=np.array(points); ax.plot(a[:,0],a[:,1],color=color,lw=1.2,**kw)

def arrow(ax,a,b,color=INK,**kw):
    ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',color=color,lw=1.1,**kw))

def box(ax,x,y,w,h,text,color=V,size=8):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.10',
                 fc='white',ec=color,lw=1.1))
    ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=size,color=INK,linespacing=1.45)

def spring(ax,x,y0,y1):
    y=np.linspace(y0+.2,y1-.2,11)
    xx=x+.18*np.array([0]+[(-1)**k for k in range(9)]+[0])
    line(ax,[(x,y0),(x,y0+.2)],V)
    line(ax,list(zip(xx,y)),V)
    line(ax,[(x,y1-.2),(x,y1)],V)

def damper(ax,x,y0,y1):
    mid=(y0+y1)/2
    line(ax,[(x,y0),(x,mid-.3)],V)
    line(ax,[(x-.28,mid-.3),(x+.28,mid-.3)],V)
    line(ax,[(x-.36,mid-.5),(x-.36,mid+.35),(x+.36,mid+.35),(x+.36,mid-.5)],V)
    line(ax,[(x,mid+.35),(x,y1)],V)

def ground(ax,x,y):
    line(ax,[(x-.5,y),(x+.5,y)],P)
    for dx in np.linspace(-.4,.4,5):line(ax,[(x+dx,y),(x+dx+.17,y+.2)],P)

def init(ax,title):
    ax.set(xlim=(0,10),ylim=(0,12));ax.axis('off')
    ax.set_title(title,fontsize=10,fontweight='bold',loc='left',pad=10)

def palm(ax,kind):
    ax.add_patch(Rectangle((2.6,4.25),4.8,.65,fc='#e8ecee',ec=P,lw=1))
    ax.add_patch(Rectangle((2.8,4.16),4.4,.16,fc='#b7dddd',ec=P,lw=.7))
    ax.plot(5,4.25,'o',color=E,ms=5,zorder=5)
    ax.text(7.5,4.55,'Palm',fontsize=8,color=P,va='center')
    ax.text(5.35,5.15,r'$p_c,\;v_c=J_c\dot q$',color=E,fontsize=8)
    line(ax,[(5,4.25),(5,5.45)],E)
    if kind=='ball':
        ax.add_patch(Circle((5,3.36),.78,fc='#fae4bd',ec=P,lw=1.2))
        line(ax,[(1.8,2.58),(8.2,2.58)],P)
        ax.text(7.1,2.78,'Table',color=P,fontsize=8)
        ax.add_patch(Arc((5,3.36),1.0,1.0,theta1=20,theta2=150,color=P,lw=1))
        arrow(ax,(4.56,3.61),(4.50,3.50),P)
        arrow(ax,(3.0,3.25),(3.9,3.25),P)
        ax.text(2.9,3.6,r'$s$: ball progress',color=P,fontsize=8,ha='right')
    elif kind=='joystick':
        ax.add_patch(Circle((5,3.91),.25,fc='#d5dce0',ec=P))
        line(ax,[(5,3.66),(4.5,2.6)],P)
        ax.add_patch(Circle((4.5,2.6),.12,fc=P))
        ground(ax,4.5,2.45)
        ax.text(5.4,3.2,'Physical joystick',fontsize=8,color=P)
        ax.add_patch(Arc((4.5,2.6),1.8,1.8,theta1=60,theta2=110,color=P,lw=1))
    else:
        ax.add_patch(Circle((5,3.91),.25,fc='#d5dce0',ec=P))
        line(ax,[(5,3.66),(5,2.7)],P)
        ground(ax,5,2.6)
        ax.text(5.55,3.2,'Physical support',fontsize=8,color=P)

def coupling(ax,support=False):
    # Mechanical spring and damper in parallel for the moving task reference.
    spring(ax,4,5.7,7.7);damper(ax,6,5.7,7.7)
    line(ax,[(4,5.7),(6,5.7)],V);line(ax,[(5,5.7),(5,5.45)],V)
    if support:
        line(ax,[(4,7.7),(4,8.15),(5,8.15),(5,8.75)],V)
        ground(ax,6,7.9);line(ax,[(6,7.7),(6,7.9)],V)
        ax.text(7,7.95,'Zero-velocity\nreference',fontsize=7.5,va='center',color=V)
    else:
        line(ax,[(4,7.7),(6,7.7)],V);line(ax,[(5,7.7),(5,8.75)],V)
    ax.text(3.4,6.7,r'$K_c$' if not support else '$K$',color=V,ha='right')
    ax.text(6.6,6.7,r'$D_c$' if not support else '$D$',color=V)
    arrow(ax,(2.6,5.8),(2.6,6.7),V)
    ax.text(2.25,6.25,r'$f_c$' if not support else '$f_v$',color=V,ha='right')

def save(fig,name):
    for ext in ['pdf','png','svg']:
        fig.savefig(OUT/f'{name}.{ext}',dpi=240,bbox_inches='tight',facecolor='white')
    plt.close(fig)

def main():
    fig,ax=plt.subplots(figsize=(3.45,4.3));init(ax,'Support loading and reorientation')
    box(ax,1,8.85,8,2.15,'Prescribed anchor '+r'$a(t)$'+'\n'+r'$f_v=K(a-p_c)-D v_c$')
    coupling(ax,True);palm(ax,'support')
    ax.text(.3,1.9,'Orientation spring + damper: follow '+r'$R_d(t)$'+'\n'
            'EIT: define VMC attachment at contact\n'
            'Loading: advance anchor; reorientation: change '+r'$R_d$',fontsize=7.8,linespacing=1.6,va='top')
    ax.text(.3,.25,'Amber: virtual components   Grey: physical setup\nBlue dot: selected centre / EIT / known attachment',fontsize=7.3,color=INK,linespacing=1.5)
    fig.tight_layout();save(fig,'vmc_support_structure')

    fig,axs=plt.subplots(1,2,figsize=(7.1,4.9))
    for ax,title,kind in zip(axs,['(a) Joystick: moving arc reference','(b) Ball: virtual carriage'],['joystick','ball']):
        init(ax,title);coupling(ax);palm(ax,kind)
    box(axs[0],.65,8.85,8.7,2.15,'Prescribed progress '+r'$\theta_d\rightarrow +12^\circ,\;-12^\circ$'+'\n'
        r'$h(\theta_d)=h_0+[L\sin\theta_d,\;0,\;L(\cos\theta_d-1)]^T$'+'\nBounded speed and acceleration',size=7.7)
    axs[0].text(5.3,8.35,r'$h(\theta_d),\;G\dot\theta_d$',color=V,fontsize=8)
    axs[0].text(.3,1.9,'Normal spring + damper: ramp 3 N preload\n'
       'Orientation spring + damper: follow $\theta_d$\nEIT: define VMC attachment at contact',fontsize=7.8,linespacing=1.6,va='top')
    box(axs[1],.65,8.85,8.7,2.15,r'$s_g\;\longrightarrow$ goal spring $\longrightarrow\;M_v,d_v$'+'\n'
       r'$M_v\ddot s=u_g-d_v\dot s-G^T f_c$'+'\n'+r'$h(s)=h_0+2t(s-s_0),\quad G=2t$',size=8)
    axs[1].text(5.3,8.35,'2:1 transmission',color=V,fontsize=8)
    line(axs[1],[(6,5.7),(9.7,5.7),(9.7,10),(9.45,10)],V,ls='--')
    arrow(axs[1],(9.7,10),(9.45,10),V)
    axs[1].text(9.45,7.7,r'Reaction $-G^Tf_c$',rotation=90,ha='center',va='center',fontsize=7.5,color=V)
    axs[1].text(.3,1.9,'Normal spring + damper: ramp 3 N preload\n'
       'Orientation spring + damper: near-horizontal palm\nEIT: filter attachment + rebase reference',fontsize=7.8,linespacing=1.6,va='top')
    fig.text(.5,.13,r'Both task couplings: $f_c=P_t[K_c(h-p_c)+D_c(v_{\mathrm{ref}}-v_c)]$',ha='center',fontsize=8.5)
    fig.text(.5,.075,r'Joint command: $\tau=J_c^T(f_c+f_n)+J_\omega^Tm_R+\mathrm{bias}-B_q\dot q$ (with limits)',ha='center',fontsize=8.5)
    fig.text(.5,.025,'Amber: virtual components   Grey: physical setup   Blue dots / links: selected attachment',ha='center',fontsize=7.5)
    fig.subplots_adjust(left=.025,right=.975,bottom=.22,top=.92,wspace=.15)
    save(fig,'vmc_task_structures')

if __name__=='__main__':main()
