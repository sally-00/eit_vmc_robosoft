# Joystick, ball rolling and tactile model switching

Executed MuJoCo study of the one-axis joystick and straight rolling protocols in
`../RoboSoft_EIT_VMC_Draft/manuscript.tex`. All task results are synthetic.

Read `report/Joystick_Ball_EIT_VMC_Simulation_Report.pdf` or `report/report.html`.
The report states the purpose, procedure, comparison, scoring and interpretation
for each experiment, and contains actual MuJoCo setup and motion images.

## Completed evaluation

540 frozen trials consumed 5,546,276 physics steps from a maximum budget of
13,560,000. Trials have finite horizons and a maximum of six engagements. Four
workers execute separate episodes; dynamics within each episode are sequential.

With touch switching, EIT and known-location joystick attachments each pass
27/27 trials, while the centre passes 9/27. The adaptive point spring also passes
27/27 and gives smaller angle error than the virtual lever. The gentle short
ball task passes 27/27 with both centre and EIT attachments. Thus these results
do not establish a universal benefit from attachment adaptation or mechanism state.

The independent support-withdrawal task reduces mean disconnected travel from
33.62 mm (timed/EIT) to 4.02 mm (touch/EIT), while increasing mean peak normal
load from 3.48 to 6.20 N. The report retains this recovery tradeoff. A lower
joystick return stiffness also exposes a lever limitation: 0/9 versus 9/9 for
the matched point spring. All six high voltage-noise diagnostic trials fail.

## Reproduction

Python 3.12 was used. The project reuses the pinned UR3e parameter files and
saved training arrays in the adjacent `../ur3e_sim` directory. The original
draft and prior report have not been replaced.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export MPLCONFIGDIR=/tmp/eit_task_mpl
python src/restore_estimator.py
python src/campaign.py --workers 4
python src/analyze.py
MUJOCO_GL=egl python src/figures.py
python src/make_report.py
```

The campaign resumes from existing traces and verifies the frozen source hashes.
The supplied evaluation is complete. Keep results and source together. A changed
controller requires a new campaign directory and frozen protocol; do not mix
its trajectories into this evaluation. `src/campaign.py --freeze` creates a new
schedule only when evaluation traces are absent.

`src/verify.py` performs contact-frame equivalence, transmission-power, mesh and
time-step checks. Its saved results are in `results/verification.json`.
`results/runtime_environment.json` records imported module versions and paths.
The present workspace run used `/home/yi/miniconda3/envs/mujoco312/bin/python`
with the additional dependencies installed under `/tmp/eit_task_deps`; this
temporary folder is unnecessary after a normal installation of requirements.
The imported NumPy module reports 2.5.3 while its distribution metadata reports
2.4.6; both are recorded in the environment artifact. A clean environment and
the saved numerical checks are appropriate for independent reproduction.

## Estimator integrity

The copied original `../ur3e_sim/results/eit_estimator.joblib` is truncated and
does not match its manifest. The new project's restored checkpoint was fitted
from the unchanged original training arrays with the original settings. Its
SHA256 exactly matches the original manifest, and its saved training, validation,
test and shift predictions have zero difference. The audit is in
`results/estimator_restoration.json`. Original files were preserved.

The controller generates synthetic FEM voltages and runs this estimator online.
The estimator predicts force as well as position; force is never supplied as task feedback.
The normal preload is a spring-reference demand. All ball controllers share the
same EIT/pose progress observations, state correction and between-stroke camera
policy. Actual sphere rotation and native slip are independent evaluation signals.

## Artifact layout

- `results/frozen_protocol.json`: paired schedule, thresholds, source hashes,
  environment and physical-step budget.
- `results/trials/trial_*.npz`: 100 Hz traces, full robot/object positions, table
  poses, original XML, events, trial configuration and metrics. `columns` names
  every trace column.
- `results/analyzed_metrics.csv`: all 540 attempted trials. Missing joystick
  scoring windows remain unavailable; their episodes remain in pass denominators.
- `results/summary.json`: trial-level counts and paired comparisons.
- `results/STATUS.json`: completed trial and physics-step counts.
- `development/`: excluded pilot traces. `dev_*.npz` retain the initial fast
  support-return implementation; `loss_v2_*.npz` use the slower selected return.
- `verification/`: excluded time-step refinement trajectories.
- `report/figures/`: setup images, mechanism diagrams, trajectories and outcome
  plots as PNG and PDF.

## Scope and buying a ball

The report recommends a smooth solid 50 mm silicone/rubber sphere as an initial
purchase specification, nominal Shore A 50-70, with known diameter, mass and
surface finish. This is a design starting point, not a validated material match.
Measure friction and indentation on the actual EIT pad and table. Supplier links
and the assumed bracket-clearance calculation are in report Section 11.

The ball is a rigid dynamic sphere with compliant numerical contact; electrical
footprint width does not derive from measured curvature, shear or deformation.
The recess is assumed, robot pose and model compensation are ideal, and arm
self-collision/full fixture clearance are unvalidated. Simulated torque commands
do not establish a physical UR3e torque interface. The draft's later two-axis
gimbal extension is not part of these one-axis protocols.
