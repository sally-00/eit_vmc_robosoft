from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
from figure_style import COLORS, VIRTUAL, CONTACT, PHYSICAL, INK, BASELINE_TEXT

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'figures';OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
col={'fixed':COLORS['centre'],'eit':CONTACT,'oracle':COLORS['oracle']}
labels={'fixed':'Fixed centre','eit':'EIT selected','oracle':'Known location'}

def arrow(ax,a,b,color='#364656',style='->'):
    ax.add_patch(FancyArrowPatch(a,b,arrowstyle=style,mutation_scale=10,lw=1.3,color=color))

def spring(ax,a,b,color=VIRTUAL):
    a=np.array(a);b=np.array(b);v=b-a;n=np.array([-v[1],v[0]]);n/=np.linalg.norm(n)
    ts=np.linspace(.15,.85,13);pts=np.array([a+t*v+n*(.06*(-1)**i) for i,t in enumerate(ts)])
    ps=np.vstack([a,a+.1*v,pts,b-.1*v,b]);ax.plot(ps[:,0],ps[:,1],color=color,lw=1.6)

# Concept figure: generated human illustration plus exact vector annotations.
fig=plt.figure(figsize=(7.1,2.65))
axs=[fig.add_axes(r) for r in [(0.005,.16,.34,.75),(.375,.16,.235,.75),(.66,.16,.335,.75)]]
for ax in axs:
 ax.set_xlim(0,4);ax.set_ylim(0,3.8);ax.set_aspect('equal');ax.axis('off')
ax=axs[0];fig.text(.175,.96,'(a) Human motivation',ha='center',fontsize=9,fontweight='bold')
img=plt.imread(ROOT/'assets'/'palm_rolling_concept.png')
ax.imshow(img,extent=(0,4,1.25,3.25),aspect='auto')
ax.annotate('Contact hidden by the palm',xy=(1.72,1.97),xytext=(2.02,.79),ha='center',fontsize=7.5,
 arrowprops={'arrowstyle':'->','color':'#364656','lw':1})
ax.text(2,.18,'Touch informs adjustment after contact',ha='center',fontsize=7.4)
ax=axs[1];fig.text(.4925,.96,'(b) Soft EIT palm',ha='center',fontsize=9,fontweight='bold')
ax.add_patch(Circle((2,2.25),1.22,fc='#e6eaed',ec='#73848b',lw=.8))
ax.add_patch(Circle((2,2.25),.94,fc='#cfe8e8',ec='#527376',lw=1))
for t in np.arange(8)*np.pi/4:
 ax.add_patch(Circle((2+.91*np.cos(t),2.25+.91*np.sin(t)),.065,fc=PHYSICAL))
ax.plot(2,2.25,'x',color=col['fixed'],ms=5,mew=1)
ax.add_patch(Circle((2.46,2.03),.13,fc=CONTACT,ec=INK,lw=.6))
ax.annotate('Contact',xy=(2.46,2.03),xytext=(3.9,1.17),ha='right',fontsize=6.8,color=CONTACT,
 arrowprops={'arrowstyle':'->','color':CONTACT,'lw':.75})
ax.text(2,.77,'8 boundary electrodes',ha='center',fontsize=7.6)
ax.text(2,.18,'Contact state + location',ha='center',fontsize=7.6,fontweight='bold')
ax=axs[2];fig.text(.8275,.96,'(c) Configure virtual mechanics',ha='center',fontsize=9,fontweight='bold')
# Raised bracket sides remain clear of the smaller contacting ball.
ax.add_patch(Rectangle((.25,2.0),3.5,.26,fc='#d5dadd',ec='#79838a',lw=.7))
ax.add_patch(Rectangle((.25,1.69),.22,.45,fc='#d5dadd',ec='#79838a',lw=.7))
ax.add_patch(Rectangle((3.53,1.69),.22,.45,fc='#d5dadd',ec='#79838a',lw=.7))
ax.add_patch(Rectangle((.47,1.92),3.06,.16,fc='#88c5c6',ec='#527376',lw=.6))
ax.add_patch(Circle((2.7,1.35),.57,fc='#d4e7e5',ec='#527376',lw=.8))
ax.plot([.15,3.85],[.78,.78],color='#7e8a90',lw=.8)
ax.plot(1.9,1.99,'x',color=col['fixed'],ms=6,mew=1.3)
ax.plot(2.7,1.92,'o',color=CONTACT,ms=4)
spring(ax,(2.7,1.92),(2.7,3.15));spring(ax,(2.7,1.92),(3.7,2.88))
ax.plot(2.7,3.15,'s',ms=3,color=VIRTUAL);ax.plot(3.7,2.88,'s',ms=3,color=VIRTUAL)
ax.text(.23,3.23,'Same task goal',fontsize=7.6)
ax.annotate('Nominal centre',xy=(1.9,1.99),xytext=(.25,2.65),fontsize=6.8,color=BASELINE_TEXT,
 arrowprops={'arrowstyle':'->','color':BASELINE_TEXT,'lw':.7})
ax.text(2,.38,'Contact event: engage components',ha='center',fontsize=7.3)
ax.text(2,.03,'Contact location: select attachment',ha='center',fontsize=7.3)
# Conceptual information flow, not a human physiological model.
for x in [.352,.625]:
 fig.add_artist(FancyArrowPatch((x,.53),(x+.025,.53),transform=fig.transFigure,
 arrowstyle='->',mutation_scale=10,lw=1,color='#4e646c'))
fig.text(.5,.045,'Adapt the interaction to the contact encountered',ha='center',fontsize=9,fontweight='bold',color=CONTACT)
fig.savefig(OUT/'framework.pdf',bbox_inches='tight');fig.savefig(OUT/'framework.png',dpi=240,bbox_inches='tight');plt.close(fig)

import sys
if '--framework-only' in sys.argv:
 print('Created framework figure');raise SystemExit(0)

DATA=ROOT.parent/'results'
if not DATA.exists(): DATA=ROOT/'evidence'
df=pd.read_csv(DATA/'analyzed_metrics.csv')
fig,axs=plt.subplots(1,3,figsize=(7.1,2.05))
for i,c in enumerate(['fixed','eit','oracle']):
 s=df[(df.stage=='S1')&(df.offset_mm.abs()>1)&(df.condition==c)]
 axs[0].bar(i,s.rotation_abs_deg.mean(),color=col[c],width=.65)
 s=df[(df.stage=='S2')&(df.condition==c)].copy();s['angle_error']=abs(s.achieved_deg-s.angle_deg)
 g=s.groupby('offset_mm').agg(success=('success','mean'),error=('angle_error','mean'))
 axs[1].plot(g.index,100*g.success,'o-',color=col[c],label=labels[c],ms=3,lw=1)
 axs[2].plot(g.index,g.error,'o-',color=col[c],ms=3,lw=1)
axs[0].set_xticks([0,1,2],['Centre','EIT','Known']);axs[0].set_ylabel('Unwanted rotation (deg)');axs[0].set_title('(a) Loading mechanism')
axs[1].set_ylabel('Task success (%)');axs[1].set_ylim(0,108);axs[1].set_title('(b) Supported reorientation');axs[1].legend(fontsize=6,loc='lower center',frameon=False)
axs[2].set_ylabel('Mean orientation error (deg)');axs[2].axhline(1,color='#888',ls='--',lw=.7);axs[2].set_ylim(0,1.08);axs[2].set_title('(c) Reaching the angle target')
for ax in axs[1:]:ax.set_xlabel('Contact offset (mm)');ax.set_xticks([-30,0,30])
fig.tight_layout(w_pad=1.2);fig.savefig(OUT/'results.pdf',bbox_inches='tight');fig.savefig(OUT/'results.png',dpi=220,bbox_inches='tight');plt.close(fig)

fig,axs=plt.subplots(1,2,figsize=(3.45,1.65))
for ax in axs:
 ax.set_xlim(0,4);ax.set_ylim(0,3.5);ax.axis('off')
 # recessed palm and bracket in section
 ax.add_patch(Rectangle((.45,2.5),3.1,.35,fc='#d5dadd',ec='#79838a'))
 ax.add_patch(Rectangle((.45,2.15),.25,.5,fc='#d5dadd',ec='#79838a'))
 ax.add_patch(Rectangle((3.3,2.15),.25,.5,fc='#d5dadd',ec='#79838a'))
 ax.add_patch(Rectangle((.7,2.35),2.6,.18,fc='#88c5c6',ec='#527376'))
 ax.plot(2,2.35,'o',ms=3,color=CONTACT)
axs[0].set_title('(a) Joystick',fontsize=8)
axs[0].add_patch(Rectangle((.4,.35),3.2,.4,fc='#eeeeee',ec='#777'))
axs[0].plot([2,2],[.75,1.95],color='#666',lw=3)
axs[0].add_patch(Circle((2,2.06),.29,fc='#ccc',ec='#666'))
arrow(axs[0],(1,1.45),(3,1.45),style='<->')
axs[0].text(2,-.05,'Target joystick input',ha='center',fontsize=6.5)
axs[1].set_title('(b) Ball rolling',fontsize=8)
axs[1].add_patch(Circle((2,1.43),.92,fc='#edddbb',ec='#9c895f'))
axs[1].plot([.1,3.9],[.5,.5],color='#777')
arrow(axs[1],(2.1,.15),(3.5,.15),color=CONTACT)
axs[1].text(2,-.2,'Target ball displacement',ha='center',fontsize=6.5)
fig.tight_layout(w_pad=.8,pad=.1);fig.savefig(OUT/'hardware.pdf',bbox_inches='tight');fig.savefig(OUT/'hardware.png',dpi=220,bbox_inches='tight');plt.close(fig)
print('Created three paper figures from geometry and saved trial metrics')
