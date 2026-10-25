# UR3e + eight-electrode EIT + virtual model control

Executed simulation project. **No hardware data or robot commands.**

Read `report/UR3e_EIT_VMC_Simulation_Report.pdf` or open `report/report.html`.
The PDF describes the models, assumptions, numerical checks, experiment design,
results and limitations. All figures are generated from specified geometry or
actual saved numerical outputs. The report distinguishes synthetic sensor
accuracy from measured EIT performance.

## Main findings

- Primary +/-5-degree supported reorientation: fixed centre 40/50 successful;
  EIT location 50/50; known location 50/50.
- At +/-10 degrees: fixed centre 6/18; EIT and known location 18/18.
- At +/-15 degrees: fixed centre 6/18; EIT and known location 15/18.
- Force feedback remains materially less accurate than location selection in
  the assumed sensor family. See the per-target errors and overshoot in S4.
- All accuracy values are synthetic. The constitutive law is not calibrated
  to the physical sensor. Eight electrodes alone do not determine that law.

The project is located at `/media/yi/Yi Cam data/09_EIT_vmc/ur3e_sim`.

## Reproduce

Python 3.12 was used. A virtual environment is recommended.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python src/eit.py
python src/run_suite.py --workers 6
python src/run_extension.py
python src/verify.py
python src/analyze.py
python src/force_baselines.py
MUJOCO_GL=egl python src/figures.py
MUJOCO_GL=egl python src/render_experiment_scenes.py
python src/draw_vmc_model.py
python src/make_report.py
```

The archive already includes completed results. `run_suite.py` resumes and
skips existing numbered trial files. To execute everything from scratch,
rename the `results` directory first; do not mix partial results from edited
physics/controller code with the saved schedule. The `report/figures` folder
can be regenerated. Rendering needs an EGL/OpenGL-capable environment; use an
appropriate MuJoCo rendering backend if EGL is unavailable.

`src/audit_events.py` is a record of a post-run bug audit, not a required step
for fresh runs: the current controller already counts false electrical
acquisition as failure. The audit examined 136 unique initial histories and
replaced 10 affected high-noise trials. It did not retune the controller or
change the primary task results. The old implementation is retained only in
`results/audit/experiments_before_event_fix.py`; do not run that version.

## Geometry

- UR3e kinematics, masses, centres of mass and inertia tensors come from the
  official Universal Robots description at the commit in `PROVENANCE.json`.
- Active EIT disk: diameter 90 mm. Eight electrode arcs at 45-degree intervals.
- Whole device: diameter 130 mm, total thickness 33.5 mm. Rear mounts to tool0.
- Device mass 0.35 kg and 3 mm front collision-layer thickness are assumptions.
- Rounded support radius 8 mm. Fixed in S2/S3, spring-supported in S1/S4.
- Collidable outer annulus produces no EIT touch observation.
- Simplified link visuals; robot self-collision/fixture clearance unvalidated.

## Electrical model and calibration needed

`src/eit.py` implements a 2-D linear FEM complete-electrode forward model, with
finite electrode arcs and contact impedance. Main results use 40 adjacent
drive/adjacent sense scalar measurements, including reciprocal redundancy.
The estimator receives normalized voltage changes, not true position.

Contact pressure is represented by an assumed Gaussian sheet-conductance
change with force saturation, material uncertainty, an 80 ms dynamic lag and
6% unloading sensitivity shift. This is not a coupled 3-D hyperelastic / AC
electrical simulation or a calibrated TouchLoop twin. The optional grounded
shunt and opposite-drive modes are exploratory capabilities, not validated
equivalent sensor implementations. Confirm the actual material/layer stack,
grounding, drive schedule, excitation, electrode dimensions and frame timing
before using this model for transfer predictions.

## Data layout

- `results/eit_{train,validation,test,shift}.npz`: normalized voltage features
  X, labels Y=(x metres, y metres, normal force N), episode and session ids.
- `results/eit_*_predictions.npz`: held-out targets and estimator predictions.
- `results/trial_metrics.csv`: 481 primary trials.
- `results/exploratory_metrics.csv`: 138 separately identified follow-ups.
- `results/trial_data/trial_*.npz`: 100 Hz trace and JSON trial description.
- `results/frozen_protocol.json`: pre-evaluation primary settings/source hashes.
- `results/exploratory_protocol.json`: later follow-up with no gain retuning.
- `results/audit/event_correction.json`: acquisition correction and audit.
- `results/verification.json`: Jacobian and time-step refinement checks.
- `report/figures/*.png` and `*.pdf`: raster and vector figures.
- `report/figures/scene_s1` through `scene_s4`: MuJoCo views reconstructed
  from saved trial states for the four full-arm experiment stages.
- `report/figures/vmc_model.{png,pdf,svg}`: mechanical schematic and control
  diagram of the implemented VMC, including the optional S4 force path.

Trace columns, in order:

| Index | Quantity | Units |
|---|---|---|
| 0 | Time | s |
| 1 | True normal contact force | N |
| 2 | Last acquired EIT force estimate | N |
| 3 | World-y component of relative orientation rotation vector | rad |
| 4 | Latched material-point displacement from its latch world position | m |
| 5 | Contact-locus migration relative to latched sensor coordinates | m |
| 6 | Cumulative relative tangential slip | m |
| 7 | Latched flag | 0/1 |
| 8 | World z of sensor-face centre | m |
| 9 | Support vertical-joint displacement (zero for fixed support) | m |
| 10-11 | Instantaneous contact x/y in sensor coordinates | m |
| 12-17 | Six robot joint positions | rad |

Core S1-S3 do not use force estimates for feedback. True force is used to
simulate the electrical response and to evaluate performance. S4 deliberately
uses known location in all conditions to isolate the force-feedback question.

Primary S2/S3 success is defined in the report and frozen protocol. For S1/S4,
the CSV `success` field only denotes a completed, non-aborted episode; it is
not a thresholded force-accuracy claim. Analyze their quantitative metrics.

## Model provenance and licenses

Only the official UR3e parameter files and xacro reference are needed. Their
BSD-3-Clause license is retained in `vendor/ur_description/LICENSE`.
The community MJCF model inspected during development was not used in the
final dynamics model and is not included in this deliverable.

The original uploaded draft is not redistributed in the project. Its equations
and role in this study are cited in the report. No physiological claim about
human touch is tested by these simulations.
