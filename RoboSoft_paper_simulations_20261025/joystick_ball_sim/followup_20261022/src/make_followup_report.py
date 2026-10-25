"""Plots and a standalone report from saved attempts only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import sys,json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Image,Table,TableStyle,PageBreak
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'report';FIG=OUT/'figures';FIG.mkdir(exist_ok=True)
sys.path.insert(0,str(ROOT.parent/'src'))
from figures import render
plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':180})
COL={'odometry':'#bf5c38','eit':'#008c85','oracle':'#455ea8','centre':'#bf5c38','latched':'#9c6dba','prescribed':'#9c6dba','carriage':'#008c85','no_reaction':'#bf5c38'}
LABEL={'odometry':'Palm odometry','eit':'EIT','oracle':'Known contact','centre':'Centre','latched':'EIT latched','prescribed':'Prescribed reference','carriage':'Carriage','no_reaction':'Reaction removed'}
df=pd.read_csv(ROOT/'evaluation/analyzed_metrics.csv');S=json.loads((ROOT/'evaluation/summary.json').read_text())
def save(fig,name):
    fig.tight_layout()
    for ext in ['png','pdf']:fig.savefig(FIG/f'{name}.{ext}',bbox_inches='tight',facecolor='white')
    plt.close(fig)
def load(row):return np.load(ROOT/'evaluation/trials'/f"trial_{int(row.trial_id):04d}.npz")
def stat(group,**kw):
    return next(r for r in S[group] if all(r.get(k)==v for k,v in kw.items()))
def nfmt(x,d=2):return 'unavailable' if x is None or pd.isna(x) else f'{x:.{d}f}'

# All feedback cases, including severe failures, are displayed.
cases=['nominal','forward_2N','forward_4N','reverse_4N','severe_6N']
case_labels=['No push','+2 N, 0.25 s','+4 N, 0.35 s','-4 N, 0.15 s','+6 N, 0.15 s']
fig,axes=plt.subplots(1,2,figsize=(10,3.6))
for j,source in enumerate(['odometry','eit','oracle']):
    ss=[stat('feedback',case=c,progress_source=source) for c in cases]
    x=np.arange(len(cases))+(j-1)*.23
    axes[0].bar(x,[r['late_error_mm'] if r['scored'] else np.nan for r in ss],width=.22,color=COL[source],label=LABEL[source])
    axes[1].bar(x,[100*r['passes']/r['n'] for r in ss],width=.22,color=COL[source])
    for xx,r in zip(x,ss):
        axes[1].text(xx,100*r['passes']/r['n']+2,f"{r['passes']}/{r['n']}",ha='center',fontsize=6.5)
        if not r['scored']:axes[0].text(xx,.03,'N/A',ha='center',rotation=90,fontsize=6)
for a in axes:a.set_xticks(range(5),case_labels,rotation=20,ha='right')
axes[0].set(ylabel='Last-second mean absolute error (mm)',title='(a) Physical ball position, common horizon');axes[0].legend(fontsize=7)
axes[1].set(ylabel='Trials meeting all criteria (%)',ylim=(0,115),title='(b) Every attempt counted')
save(fig,'feedback')

# Traces use a prespecified first evaluation seed/offset, not best-case selection.
selection=df[(df.group=='feedback')&(df.case=='forward_4N')&(df.offset==.01)&(df.seed==1600)]
fig,axes=plt.subplots(2,2,figsize=(10,6))
for _,r in selection.iterrows():
    z=load(r);tr=z['trace'];t=tr[:,0]-r.evaluation_start_s;c=COL[r.progress_source]
    axes[0,0].plot(t,1000*(tr[:,4]-tr[:,27]),c=c,label=LABEL[r.progress_source])
    axes[0,1].plot(t,1000*(tr[:,13]-tr[:,4]),c=c)
    axes[1,0].plot(t,tr[:,1],c=c)
    axes[1,1].plot(t,1000*tr[:,16],c=c)
for a in axes.flat:a.axvspan(2,2.35,color='#dddddd',alpha=.6);a.set_xlabel('Time after engagement (s)');a.set_xlim(0,8)
axes[0,0].set(ylabel='True position minus target (mm)',title='(a) Recovery after a forward push');axes[0,0].legend(fontsize=7)
axes[0,1].set(ylabel='Progress estimate minus truth (mm)',title='(b) State estimation error')
axes[1,0].set(ylabel='Pad normal force (N)',title='(c) Independent load measurement')
axes[1,1].set(ylabel='Accumulated top-contact slip (mm)',title='(d) Native simulated slip')
save(fig,'feedback_traces')
fig,axes=plt.subplots(2,3,figsize=(10,5.8))
for i,source in enumerate(['odometry','eit']):
    r=selection[selection.progress_source==source].iloc[0];z=load(r);tr=z['trace']
    for j,age in enumerate([1.8,2.4,7.8]):
        ix=np.argmin(abs(tr[:,0]-r.evaluation_start_s-age));axes[i,j].imshow(render(z,ix,True));axes[i,j].axis('off')
        axes[i,j].set_title(f'{LABEL[source]}, t={age:.1f} s\nerror={(tr[ix,4]-tr[ix,27])*1000:+.2f} mm',fontsize=9)
save(fig,'setup_states')

fig,axes=plt.subplots(1,3,figsize=(10,3.4))
for attachment in ['centre','latched','eit','oracle']:
    vals=[stat('attachment',orientation_k=k,attachment=attachment) for k in [.3,1.,3.]]
    for ax,field in zip(axes,['peak_pitch_deg','late_error_mm','peak_force_N']):
        ax.plot([.3,1,3],[r[field] for r in vals],'o-',c=COL[attachment],label=LABEL[attachment])
for a in axes:a.set_xscale('log');a.set_xticks([.3,1,3],['0.3','1','3']);a.set_xlabel('Orientation stiffness (Nm/rad)')
axes[0].set(ylabel='Mean peak palm pitch (deg)',title='(a) Alignment during migration');axes[0].legend(fontsize=6.5)
axes[1].set(ylabel='Late error (mm)',title='(b) Endpoint accuracy')
axes[2].set(ylabel='Mean peak pad force (N)',title='(c) Load tradeoff')
save(fig,'attachments')

fig,axes=plt.subplots(1,3,figsize=(10,3.5))
for j,policy in enumerate(['prescribed','carriage','no_reaction']):
    vals=[stat('dynamics',case=c,dynamics_label=policy) for c in ['nominal','resistance','stop_restart']]
    for ax,field in zip(axes,['late_error_mm','integrated_error_mm_s','peak_force_N']):
        ax.bar(np.arange(3)+(j-1)*.24,[r[field] for r in vals],width=.23,color=COL[policy],label=LABEL[policy])
for a in axes:a.set_xticks(range(3),['Nominal','Resistance','Stop/restart'],rotation=20,ha='right')
axes[0].set(ylabel='Late error (mm)',title='(a) Common scoring horizon');axes[0].legend(fontsize=6.5)
axes[1].set(ylabel='Integrated error (mm s)',title='(b) Includes transient error')
axes[2].set(ylabel='Mean peak pad force (N)',title='(c) Contact load')
save(fig,'dynamics')

fig,axes=plt.subplots(1,2,figsize=(9.8,3.7))
biases=[-.005,0,.005];delays=[0,.1,.25,.5]
mat=np.array([[stat('quality',progress_bias=b,progress_delay=d)['late_error_mm'] for d in delays] for b in biases],float)
im=axes[0].imshow(mat,aspect='auto',origin='lower',cmap='YlOrRd',vmin=0,vmax=max(6,np.nanmax(mat)))
for i,b in enumerate(biases):
 for j,d in enumerate(delays):
    r=stat('quality',progress_bias=b,progress_delay=d)
    axes[0].text(j,i,f"{nfmt(r['late_error_mm'])} mm\n{r['passes']}/{r['n']} pass",ha='center',va='center',fontsize=8)
axes[0].set_xticks(range(4),[0,100,250,500]);axes[0].set_yticks(range(3),[-5,0,5]);axes[0].set(xlabel='Additional progress delay (ms)',ylabel='Added world-position bias (mm)',title='(a) Error and all-criteria completion')
fig.colorbar(im,ax=axes[0],label='Late error (mm)',fraction=.045)
vals=S['dropout'];axes[1].plot([r['dropout_duration'] for r in vals],[r['late_error_mm'] for r in vals],'o-',c=COL['eit'])
for r in vals:axes[1].annotate(f"{r['passes']}/{r['n']}",(r['dropout_duration'],r['late_error_mm']),xytext=(0,7),textcoords='offset points',ha='center')
axes[1].set(xlabel='Progress-feedback dropout duration (s)',ylabel='Late error (mm)',title='(b) Contact events remain available');axes[1].margins(y=.3)
save(fig,'quality')

styles=getSampleStyleSheet()
styles['BodyText'].fontSize=9.5;styles['BodyText'].leading=13
styles['Title'].fontSize=22;styles['Title'].leading=26
story=[]
def p(text):story.append(Paragraph(text,styles['BodyText']));story.append(Spacer(1,7))
def h(text):story.append(Paragraph(text,styles['Heading2']));story.append(Spacer(1,5))
def fig(name,width=490):
    from PIL import Image as PILImage
    im=PILImage.open(FIG/f'{name}.png');w,h=im.size
    story.append(Image(str(FIG/f'{name}.png'),width=width,height=width*h/w));story.append(Spacer(1,8))
def table(rows,widths=None):
    data=[[Paragraph(str(v),styles['BodyText']) for v in row] for row in rows]
    t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e8eeee')),('VALIGN',(0,0),(-1,-1),'TOP'),('LINEBELOW',(0,0),(-1,0),.5,colors.grey),('BOTTOMPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),5)]));story.append(t)
def new():story.append(PageBreak())
a=stat('feedback',case='forward_4N',progress_source='odometry');b=stat('feedback',case='forward_4N',progress_source='eit')
l=stat('attachment',orientation_k=.3,attachment='latched');u=stat('attachment',orientation_k=.3,attachment='eit');c=stat('attachment',orientation_k=.3,attachment='centre')
story.append(Paragraph('Ball rolling: what EIT adds',styles['Title']));p('Simulation follow-up • 22 October 2026')
p(f"The study contains {len(df)} evaluation attempts with frozen schedules and sensor seeds excluded from 68 development attempts. It tests progress feedback, contact migration, reference dynamics, and sensing quality. All results are synthetic MuJoCo/FEM simulations; the physical EIT palm has not been validated by this study.")
h('Main findings')
table([['Question','Result','Interpretation'],['Does ongoing EIT position feedback help?',f"After a +4 N, 0.35 s push, late error is {a['late_error_mm']:.2f} mm with palm odometry and {b['late_error_mm']:.2f} mm with EIT. Each condition has {a['n']} trials.",'EIT corrects progress after slip. Both conditions retain contact sensing and centre attachment.'],['Does updating the attachment help?',f"At 0.3 Nm/rad, mean peak pitch is {l['peak_pitch_deg']:.2f}° for a latched EIT attachment and {u['peak_pitch_deg']:.2f}° for an updated one.",'Contact migration makes continuous updates relevant to palm alignment. Load and completion must be considered separately.'],['Is the carriage generally better?', 'Prescribed reference, carriage, and carriage without reaction are tested at common physical observation times.','The results below determine the benefit; no general mechanism advantage is assumed.']],[150,190,150])
h('What this can strengthen in the paper')
p('The strongest candidate is EIT correction of virtual task state after an unobserved disturbance, together with attachment updates that limit unwanted palm rotation. This extends the evidence beyond initial contact detection. A benefit also obtained with the prescribed-reference controller supports the sensing contribution rather than proving a unique advantage of carriage dynamics.')
p('The full electrical pipeline runs online: synthetic 40-channel FEM voltages feed the restored 100-tree Extra Trees position estimator. Its force output is excluded from feedback. No new model is trained for these experiments.')

new();h('1. Ongoing progress feedback after a disturbance')
p('<b>Why.</b> Palm travel predicts ball travel only while the assumed rolling relation remains accurate. External forces and slip can break that relation. This experiment tests whether tactile position measurements correct the resulting task-state error.')
p('<b>What and how.</b> Request a 20 mm roll; apply a horizontal ball-body force two seconds after engagement without notifying the controller. Compare palm odometry, decoded EIT contact position transformed into world coordinates, and true contact location at the same sample times. All have centre attachment, contact events, carriage gains, limits, and camera initialization in common. Two landing offsets (0 and +10 mm) and four sensor seeds give eight trials per condition.')
fig('feedback')
p('<b>Scoring.</b> Run for eight seconds after initial engagement. Score the mean absolute true ball error and speed in the final second. Require error ≤5 mm, speed ≤4 mm/s, cross-path error ≤5 mm, peak pad force ≤12 N, rim force &lt;0.2 N, and no protective stop or premature engagement. Guarded attempts remain failures; missing late windows are not imputed as zero.')
p(f"<b>Forward disturbance.</b> With +4 N for 0.35 s, completion is {a['passes']}/{a['n']} for odometry and {b['passes']}/{b['n']} for EIT. Mean peak normal loads are {a['peak_force_N']:.2f} and {b['peak_force_N']:.2f} N. The accuracy improvement is {100*(1-b['late_error_mm']/a['late_error_mm']):.1f}%. Nominal, reverse, and severe conditions remain visible to bound this finding.")

new();h('2. Why the feedback result occurs')
fig('feedback_traces')
p('The example is the first evaluation seed (1600), +10 mm landing, and +4 N push. The grey interval is the imposed force. Actual slip is measured from native contact kinematics; it is not commanded as a displacement. A persistent odometry error can remain after the ball stops. EIT updates the progress estimate and the virtual coordinate, with reference rebasing to preserve the instantaneous coupling extension.')
p(f"Across forward-push trials, accumulated upper-contact slip averages {a['top_slip_mm']:.2f} mm with odometry and {b['top_slip_mm']:.2f} mm with EIT. EIT corrects the consequence of slip; this does not establish slip prevention. The controller receives no physical ball position, rotation, external-force value, or disturbance timestamp.")
pp0=stat('feedback_point',progress_source='odometry');pp1=stat('feedback_point',progress_source='eit')
p(f"The prescribed-reference check gives {pp0['late_error_mm']:.2f} mm with odometry and {pp1['late_error_mm']:.2f} mm with EIT, with completions {pp0['passes']}/{pp0['n']} and {pp1['passes']}/{pp1['n']}. This identifies whether the sensing benefit extends beyond the virtual carriage.")

new();h('3. Simulated physical setup and motion')
fig('setup_states')
p('These are actual saved MuJoCo states for the same two feedback conditions, before the push, shortly after it, and near the final scoring window. The palm is above a free sphere on a horizontal table. The images show the modeled geometry; endpoint differences are quantified by the independent position traces rather than inferred from image appearance.')
p('The assumed active pad diameter is 90 mm. The housing opening radius is 47 mm with a 3 mm recess. The sphere diameter is 50 mm and mass approximately 72 g. Nominal friction is 0.7 and preload demand is 3 N. The arm runs with ideal torque actuation and exact bias compensation at 1 ms. No regrasp occurs in the short-stroke comparisons; geometric limits are reported separately.')

new();h('4. Attachment updates during contact migration')
p('<b>Why.</b> A contact point latched at engagement can become stale as the ball rolls across the palm. A virtual force acting at that stale point produces a different moment. This experiment tests palm alignment and task error as orientation compliance changes.')
p('<b>What and how.</b> Hold EIT progress feedback and event handling fixed. Compare centre attachment, EIT attachment latched once, EIT attachment updated during rolling, and updated true contact. Cross +10 and +25 mm offsets, orientation stiffnesses 0.3/1/3 Nm/rad, and four new sensor seeds. Every comparison matches stiffness, damping, and limits.')
fig('attachments')
p(f"At 0.3 Nm/rad, mean peak pitch is {c['peak_pitch_deg']:.2f}° for centre, {l['peak_pitch_deg']:.2f}° for latched EIT, and {u['peak_pitch_deg']:.2f}° for updated EIT. The update reduces pitch by {100*(1-u['peak_pitch_deg']/l['peak_pitch_deg']):.1f}% relative to latching. Corresponding mean peak loads are {c['peak_force_N']:.2f}, {l['peak_force_N']:.2f}, and {u['peak_force_N']:.2f} N. These are alignment and load measurements; no new pitch success threshold was selected after seeing results.")
p('A separate -25 mm initial offset exposes the finite pad boundary during a +20 mm continuous roll. Its failures are retained and cannot be repaired merely by estimating the contact more accurately. Regrasp would require a different task protocol.')

new();h('5. Does carriage reaction improve performance?')
p('<b>Why.</b> The previous endpoint comparison stopped partly according to virtual-state velocity, which differed between controllers. Common physical scoring now tests whether that earlier advantage persists.')
p('<b>What and how.</b> Compare the prescribed reference, carriage, and the same carriage with its coupling-reaction term removed. Test nominal rolling, temporary spring resistance (2.5–3.3 s after engagement), and a 10 mm target followed by a 20 mm target at 3.5 s. Four new seeds per cell; common EIT progress, centre attachment, coupling gains, and force/speed limits. Internal parameters are inherited, with no claim of optimal tuning.')
fig('dynamics')
rows=[['Condition','Prescribed error','Carriage error','Reaction removed']]
for case in ['nominal','resistance','stop_restart']:
 rows.append([case]+[nfmt(stat('dynamics',case=case,dynamics_label=m)['late_error_mm'])+' mm' for m in ['prescribed','carriage','no_reaction']])
table(rows,[120,125,120,125])
p('Removing reaction is a diagnostic within the carriage. Its result does not by itself show superiority over the prescribed reference. Integrated error includes transients; peak load is reported alongside final accuracy. These comparisons should replace a broad claim that the ball mechanism is inherently better.')

new();h('6. Sensing quality versus task performance')
p('<b>Why.</b> A useful training or acquisition requirement must depend on closed-loop consequences. This study isolates errors in the ball-progress stream while retaining the same EIT contact detector and centre attachment.')
p('<b>What and how.</b> Under the +4 N, 0.35 s push at +10 mm landing, add world-position biases of -5/0/+5 mm and transport delays of 0/100/250/500 ms. Use three new sensor seeds per cell. Convert contact position with the pose at sampling before delay. Separately suppress progress updates for 0.5/1/2 s beginning at the push; contact events continue.')
fig('quality')
p('Cell values are late error and completion counts. A large bias can produce a stable but incorrect endpoint; delay can worsen transients or destabilize correction. Recovery after a temporary missing progress stream differs from detecting physical contact loss. This limited grid supports task-specific guidance, not a universal EIT accuracy or bandwidth specification.')
p('Training should cover curved and migrating contacts if their measured errors exceed the useful region. Timestamp alignment and acquisition speed should be prioritized when delay is limiting. Real curved-object calibration is still required before overlaying a physical sensor on this map. No physical sensing operating point is claimed here.')

new();h('7. Provenance, checks, and claim limits')
audit=json.loads((ROOT/'evaluation/audit.json').read_text())
p(f"The evaluation contains {audit['attempts']} attempted trials and {audit['physical_steps']:,} physics steps. Saved configurations match the frozen schedule, the recorded source hashes match, and the trajectory audit reports {len(audit['issues'])} issues. Baseline equivalence with all new options disabled gives zero difference in original trace columns and robot/object positions.")
p('The initial 48 pilots used seed 1500, and 20 force-screen pilots used seed 1501. Evaluation selection followed these pilots. Evaluation seeds are 1600–1603 for progress, 1700–1703 for attachments, 1800–1803 for dynamics, and 1900–1902 for quality. They vary synthetic electrical sessions; counts are descriptive rather than independent manufactured-sensor evidence.')
p('The table-translation pilot produced identical trajectories to nominal because moving the mocap position did not impose the intended horizontal contact-surface velocity. It is retained as an ineffective perturbation and excluded from disturbance claims. Gentle low-friction pilots also showed no material benefit. Severe-force and edge-contact failures are retained in the evaluation.')
p('The synthetic electrical footprint is not calibrated for sphere curvature, indentation, shear, or physical material drift. Force prediction remains outside feedback. Native rigid-body contact approximates a soft interface. Full arm clearance, hardware torque-interface feasibility, and physical bracket dimensions remain unvalidated.')
h('Recommended physical demonstration')
p('Begin with an occluded short roll and a measured, repeatable forward push. Compare ongoing EIT progress against palm-motion odometry while sharing contact detection, initial camera information, attachment, and controller gains. Record independent ball motion, rotation, and contact load. Then compare latched and continuously updated attachments at matched orientation compliance. Retain the prescribed-reference control to show which benefits arise from sensing and which from mechanism dynamics.')
h('Files')
p('evaluation/protocol.json fixes the schedule and sources; evaluation/trials contains every saved state and trace; analyzed_metrics.csv and summary.json contain all-attempt results; audit.json records integrity checks. EVALUATION_DESIGN.md documents selection and scoring. The original 540-trial campaign is a separate evidence set.')

def footer(canvas,doc):
    canvas.setFont('Helvetica',8);canvas.setFillColor(colors.grey);canvas.drawString(42,25,'Synthetic simulation • Ball rolling follow-up • 22 October 2026');canvas.drawRightString(552,25,str(doc.page))
SimpleDocTemplate(str(OUT/'Ball_Rolling_Followup_Report.pdf'),pagesize=(595,842),rightMargin=42,leftMargin=42,topMargin=40,bottomMargin=40).build(story,onFirstPage=footer,onLaterPages=footer)
print(OUT/'Ball_Rolling_Followup_Report.pdf')
