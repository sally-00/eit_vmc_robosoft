"""Main task figure: selected task comparisons from archived evidence; no simulations run."""
from pathlib import Path
import hashlib,json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from figure_style import COLORS
ROOT=Path(__file__).resolve().parent
E=ROOT/'evidence';F=ROOT/'figures'
inputs=[E/'task_simulations/selected_comparisons.csv',E/'task_simulations/original_540_metrics.csv',E/'ball_followup/analyzed_metrics.csv',E/'ball_followup/summary.json']
j=pd.read_csv(inputs[0]);old=pd.read_csv(inputs[1]);ball=pd.read_csv(inputs[2]);summary=json.loads(inputs[3].read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.titlesize':9,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(2,2,figsize=(7.1,4.9));axs=axes.ravel()
col=COLORS
for a,label,marker in [('centre','Centre','s'),('eit','EIT','o')]:
 rows=j[(j.task=='joystick')&(j.attachment==a)]
 g=rows.groupby('offset').success.agg(['sum','count','mean'])
 assert len(rows)==27 and all(g['count']==9)
 axs[0].plot(g.index*1000,100*g['mean'],marker+('--' if a=='centre' else '-'),color=col[a],label=label,ms=5 if a=='centre' else 3,mfc='white' if a=='centre' else col[a])
 for x,row in g.iterrows():
  if a=='eit' or x!=0:axs[0].text(x*1000,100*row['mean']+(6 if a=='eit' else 7),f"{int(row['sum'])}/9",ha='center',fontsize=7)
axs[0].set(xlabel='Initial lateral offset (mm)',ylabel='Task success (%)',xticks=[-25,0,25],ylim=(-5,119),title='(a) Joystick: contact attachment')
axs[0].legend(frameon=False,loc='center',fontsize=7)
records={}
def bars(ax,df,key,categories,labels,metric,title,ylabel,expected):
 means=[];maximum=0
 for i,c in enumerate(categories):
  rows=df[df[key]==c].sort_values(['offset','seed']);v=rows[metric].to_numpy()
  assert len(v)==expected and np.isfinite(v).all(),(title,c,len(v))
  means.append(float(v.mean()));maximum=max(maximum,float(v.max()))
  ax.bar(i,v.mean(),color=col[c],width=.58,edgecolor="none",linewidth=0)
  ax.scatter(i+np.linspace(-.16,.16,len(v)),v,s=12,facecolor='white',edgecolor='#333333',linewidth=.5,zorder=3)
  records[title+' / '+c]={'n':len(v),'mean':float(v.mean()),'metric':metric,'trial_ids':rows.trial_id.tolist()}
 ax.set(xticks=range(len(categories)),xticklabels=labels,ylabel=ylabel,title=title,ylim=(0,maximum*1.25+.05))
 for i,v in enumerate(means):ax.text(i,maximum*1.12+.02,f'{v:.2f}',ha='center',fontsize=8,fontweight='bold')
feedback=ball[(ball.group=='feedback')&(ball['case']=='forward_4N')]
bars(axs[1],feedback,'progress_source',['odometry','eit','oracle'],['Robot motion\nestimate','EIT position\nestimate','True contact\n(reference)'],'late_error_mm','(b) Ball: correction after a push','Final ball position error (mm)',8)
attach=ball[(ball.group=='attachment')&np.isclose(ball.orientation_k,.3)]
bars(axs[2],attach,'attachment',['centre','latched','eit','oracle'],['Palm\ncentre','Initial EIT\ncontact','Updated EIT\ncontact','True contact\n(reference)'],'peak_pitch_deg','(c) Ball: moving contact point','Maximum palm tilt (deg)',8)
axs[2].tick_params(axis='x', labelsize=7)
loss=old[(old.group=='B4')&old.policy.isin(['timed_eit','event_eit'])]
bars(axs[3],loss,'policy',['timed_eit','event_eit'],['Scheduled\ncontrol','Detect loss\nand recover'],'disconnected_motion_mm','(d) Ball: contact-loss recovery','Movement without contact (mm)',6)
axs[1].text(.5,-.30,'Centre attachment; 8/8 complete in each condition',transform=axs[1].transAxes,ha='center',fontsize=7)
axs[2].text(.5,-.29,r'$K_\theta=0.3$ Nm/rad; shared EIT progress; 8/8 each',transform=axs[2].transAxes,ha='center',fontsize=7)
axs[3].text(.5,-.30,'Completion: 1/6 → 6/6; peak load: 3.48 → 6.20 N',transform=axs[3].transAxes,ha='center',fontsize=7)
fig.subplots_adjust(left=.085,right=.99,top=.94,bottom=.13,wspace=.34,hspace=.75)
for ext in ['pdf','png','svg']:fig.savefig(F/f'main_task_results.{ext}',dpi=240,bbox_inches='tight')
plt.close(fig)
audit={'figure':'main_task_results','simulation_rerun':False,'sources':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},'bar_means_and_trials':records,'presentation':'Bars are means; white markers are individual synthetic trials. No population-level confidence intervals.','omitted_panels':'Earlier nominal ball attachment and stop-restart dynamics panels; limitations retained in manuscript, complete evidence unchanged.'}
(E/'main_task_figure_provenance.json').write_text(json.dumps(audit,indent=2))
print(json.dumps(records,indent=2))
