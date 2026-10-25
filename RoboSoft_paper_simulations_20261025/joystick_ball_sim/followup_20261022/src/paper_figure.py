from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
s=json.loads((ROOT/'evaluation/summary.json').read_text())
def get(g,**kw):return next(r for r in s[g] if all(r[k]==v for k,v in kw.items()))
plt.rcParams.update({'font.size':8,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,3,figsize=(10.5,3.1))
v=[get('feedback',case='forward_4N',progress_source=x)['late_error_mm'] for x in ['odometry','eit','oracle']]
axes[0].bar(range(3),v,color=['#bf5c38','#008c85','#455ea8']);axes[0].set_xticks(range(3),['Palm\nodometry','EIT','Known\ncontact']);axes[0].set(ylabel='Late position error (mm)',title='(a) Progress feedback after a push',ylim=(0,3.3))
for i,y in enumerate(v):axes[0].text(i,y+.07,f'{y:.2f}',ha='center')
v=[get('attachment',orientation_k=.3,attachment=x)['peak_pitch_deg'] for x in ['centre','latched','eit','oracle']]
axes[1].bar(range(4),v,color=['#bf5c38','#9c6dba','#008c85','#455ea8']);axes[1].set_xticks(range(4),['Centre','EIT\nlatched','EIT\nupdated','Known\ncontact']);axes[1].set(ylabel='Mean peak palm pitch (deg)',title='(b) Migration at 0.3 Nm/rad',ylim=(0,4.7))
for i,y in enumerate(v):axes[1].text(i,y+.07,f'{y:.2f}',ha='center')
v=[get('dynamics',case='stop_restart',dynamics_label=x)['maximum_overshoot_mm'] for x in ['prescribed','carriage','no_reaction']]
axes[2].bar(range(3),v,color=['#9c6dba','#008c85','#bf5c38']);axes[2].set_xticks(range(3),['Prescribed\nreference','Carriage','Reaction\nremoved']);axes[2].set(ylabel='Mean maximum overshoot (mm)',title='(c) Stop–restart reference dynamics',ylim=(0,3.3))
for i,y in enumerate(v):axes[2].text(i,y+.07,f'{y:.2f}',ha='center')
fig.tight_layout()
for ext in ['pdf','png','svg']:fig.savefig(ROOT/'report/figures'/f'paper_followup.{ext}',dpi=200,bbox_inches='tight')
