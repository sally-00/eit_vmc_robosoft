from pathlib import Path
import json
from run_study import base
ROOT=Path(__file__).resolve().parents[1]
rows=[]
def add(group,case,**kw):rows.append(base(group=group,case=case,**kw))
# Discovery and confirmation share controller parameters, not sensor seeds.
scenarios=[('nominal',[]),('forward_2N',[dict(start=2.,duration=.25,force=2.)]),
 ('forward_4N',[dict(start=2.,duration=.35,force=4.)]),
 ('reverse_4N',[dict(start=2.,duration=.15,force=-4.)]),
 ('severe_6N',[dict(start=2.,duration=.15,force=6.)])]
for case,pulses in scenarios:
 for offset in [0.,.01]:
  for seed in range(1600,1604):
   for source in ['odometry','eit','oracle']:
    add('feedback',case,offset=offset,seed=seed,progress_source=source,disturbances=pulses)
for offset in [0.,.01]:
 for seed in range(1600,1604):
  for source in ['odometry','eit']:
   add('feedback_point','forward_4N',offset=offset,seed=seed,progress_source=source,
       mechanism='point',disturbances=[dict(start=2.,duration=.35,force=4.)])
for offset in [.01,.025]:
 for k in [.3,1.,3.]:
  for seed in range(1700,1704):
   for attachment in ['centre','latched','eit','oracle']:
    add('attachment','migration',offset=offset,orientation_k=k,seed=seed,attachment=attachment)
for seed in range(1700,1704):
 for attachment in ['centre','latched','eit','oracle']:
  add('attachment_boundary','negative_edge',offset=-.025,orientation_k=3.,seed=seed,attachment=attachment)
for case,extra in [('nominal',{}),('resistance',{'resistance':True}),
 ('stop_restart',{'goal_schedule':[dict(start=0.,distance=.01),dict(start=3.5,distance=.02)]})]:
 for seed in range(1800,1804):
  for mech,scale in [('point',1.),('mechanism',1.),('mechanism',0.)]:
   add('dynamics',case,seed=seed,offset=.01,mechanism=mech,reaction_scale=scale,**extra)
for bias in [-.005,0.,.005]:
 for delay in [0.,.1,.25,.5]:
  for seed in range(1900,1903):
   add('quality','forward_4N',seed=seed,offset=.01,progress_bias=bias,progress_delay=delay,
       disturbances=[dict(start=2.,duration=.35,force=4.)])
for duration in [.5,1.,2.]:
 for seed in range(1900,1903):
  add('dropout','forward_4N',seed=seed,offset=.01,progress_dropouts=[[2.,2.+duration]],
      dropout_duration=duration,disturbances=[dict(start=2.,duration=.35,force=4.)])
(ROOT/'evaluation_schedule.json').write_text(json.dumps(rows,indent=2))
(ROOT/'EVALUATION_DESIGN.md').write_text('''# Frozen follow-up evaluation design

Selected after 48 initial pilots (seed 1500) and 20 pulse-screen pilots (seed 1501).
Evaluation seeds are disjoint: 1600–1603 for progress feedback, 1700–1703 for
attachments, 1800–1803 for dynamics, and 1900–1902 for quality. Controller gains
are inherited; there is no tuning on evaluation seeds. All attempted trials are
retained, including severe pulses and the negative-edge clearance condition.

Feedback compares centre-attached carriages with odometry, EIT, or true-contact
progress. Nominal, forward pulses, reverse pulses, and a severe pulse characterize
where sensing does and does not help. A matched point-controller check tests
whether the benefit extends beyond the carriage. Pulse times are relative to
engagement; the controller receives no disturbance flag. This is externally
applied body force, not an imposed displacement or a desired trajectory change.

Attachment tests hold EIT progress and contact events fixed. They compare centre,
EIT latched at acquisition, EIT updated during rolling, and updated true contact.
Orientation stiffness spans 0.3, 1, and 3 Nm/rad; gains are matched within each
comparison. Report palm pitch, load, tracking and all-trial success separately.
Do not convert a retrospectively chosen pitch tolerance into a success claim.
Negative-edge trials characterize geometric limits of the continuous stroke.

Dynamics compare prescribed-reference spring–damper, carriage, and carriage
without coupling reaction. Common observation times, information, coupling and
force/speed limits remove the previous virtual-speed stopping confound. Internal
parameters are inherited and not claimed to be optimally or equally tuned.

Quality varies only world-progress observation bias and transport delay, with
contact events and centre attachment common. Positions are transformed with pose
at sampling before queuing, preventing a delayed-frame coordinate error. Dropout
suppresses progress corrections while contact detection continues. This is a
partial sensing study, not a calibrated model of real sensor delay, detachment,
or curved-object error.

The initial table-translation pilot changed mocap position without surface
velocity in the contact solve, producing identical trajectories to nominal. It
is retained as an ineffective perturbation, excluded from physical disturbance
claims, and not carried into this evaluation. Nominal low-friction pilots also
showed no material effect at the gentle commanded speed.
''')
print(len(rows),'trials')
