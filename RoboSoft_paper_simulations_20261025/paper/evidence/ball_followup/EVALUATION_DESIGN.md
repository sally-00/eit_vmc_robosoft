# Frozen follow-up evaluation design

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
