# Ball rolling follow-up, 22 October 2026

This isolated study explores three effects: ongoing tactile progress feedback,
contact attachment during migration, and virtual-carriage reaction dynamics.
The original 540-trial campaign and its source remain intact.

The extended trial code reuses the original UR3e scene, FEM voltage generator,
and restored 100-tree Extra Trees estimator. EIT force estimates do not enter
control. New runs use synthetic electrical measurements and native MuJoCo ball
contact, rotation, and slip. An oracle condition is an explicitly labelled
contact-location reference, not a learned estimator.

The progress comparison gives every condition the same tactile contact events
and a centre attachment. Odometry infers ball travel as half the measured palm
travel from the initial camera position; EIT updates ball progress from decoded
contact position and palm pose; oracle substitutes true contact location at the
same sample times. This isolates ongoing tactile position information. It is
not a comparison against a robot with no tactile contact detection.

A common observation window replaces the original controller-dependent stop:
run for eight seconds after initial engagement, and score the final second.
All attempted trials enter the completion denominator. A trial passes with mean
absolute true goal error <=5 mm, mean absolute ball speed <=4 mm/s, cross-path
error <=5 mm, peak pad force <=12 N, peak rim force <0.2 N, and no protective
termination or premature engagement. Regrasp is disabled in these short-stroke
comparisons so every controller receives the same continuous observation period.
Guards remain 15 N pad force, 1 N rim force, and 5 rad/s joint speed.

Development trials use seed 1500. Any later evaluation schedule is frozen
separately after inspecting development, with disjoint sensor seeds. All pilot
attempts and unfavourable outcomes remain available. Metrics are descriptive
simulation results, not physical reliability estimates.

The extension reproduces the original controller exactly with new options off:
verification/baseline_equivalence.json records zero difference in all original
trace columns and robot/object positions for a representative episode.

## Completed results

Read [Ball_Rolling_Followup_Report.pdf](report/Ball_Rolling_Followup_Report.pdf).
The eight-page report includes the purpose, procedure, comparisons, results,
interpretation, actual saved MuJoCo views, and limitations for every study.

329 evaluation attempts consumed 2,709,669 physics steps. All attempted trials
are retained. The analysis/configuration/source audit reports zero issues.
There are also 68 excluded development attempts, four excluded half-step checks,
and the original-versus-extended baseline pair.

- Progress feedback: after +4 N for 0.35 s, late error falls from 2.729 to 0.412 mm
  (84.9%) with EIT; 8/8 paired errors improve. Both conditions pass 8/8 and mean
  peak normal loads are 3.607 N. The prescribed-reference controller also benefits
  (2.225 to 0.463 mm), so this isolates sensing rather than unique carriage benefit.
- Migration: at 0.3 Nm/rad orientation stiffness, updating the EIT attachment
  reduces mean peak pitch from 4.009 to 1.917 degrees relative to latching (52.2%).
  All compared conditions pass; the effect is much smaller at 3 Nm/rad.
- Dynamics: in stop–restart, the carriage reduces mean overshoot from 0.989 to
  0.138 mm and late error from 0.332 to 0.108 mm versus the prescribed reference.
  Integrated error rises from 10.116 to 13.035 mm s; this is a tradeoff.
- Quality: at zero added delay, either sign of 5 mm progress bias leaves 1/3
  passing. Zero added bias passes 3/3 at each added delay through 500 ms. Delay
  results are nonmonotonic; this slow finite-duration task does not specify a
  universal sensor latency requirement. Progress-only dropouts all recover.
- Limits: all 24 severe-push and 16 negative-edge attempts terminate. The table
  translation pilot was an ineffective perturbation and supplies no moving-table
  evidence. The nominal low-friction pilot also showed no material effect.

In the four time-step checks, refining 1 ms to 0.5 ms preserves all outcomes;
maximum changes are 0.00493 mm late error, 0.00243 degrees pitch, and 0.00926 N load.
No real sensor data were collected and no model was retrained.

## Reproduction and provenance

Use the same Python environment as the parent project. Example environment:
`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MPLCONFIGDIR=/tmp/mpl_eit
PYTHONPATH=/tmp/eit_task_deps`; Python is
`/home/yi/miniconda3/envs/mujoco312/bin/python`. Run from the parent project:

```bash
python followup_20261022/src/run_study.py --name evaluation --schedule followup_20261022/evaluation_schedule.json
python followup_20261022/src/analyze_followup.py
MUJOCO_GL=egl python followup_20261022/src/finalize_report.py
python followup_20261022/src/paper_figure.py
```

The runner resumes saved attempts and verifies the frozen source hashes. Do not
regenerate the schedule in an existing evaluation directory. `protocol.json`
freezes every configuration and source dependency. `src/finalize_report.py`
adds the final interpretation to the report generator while preserving the
version recorded by the time-step-check protocol. Reporting changes do not
change simulation code or trajectories.

`evaluation/analyzed_metrics.csv`, `summary.json`, and `audit.json` are the
analysis artifacts; every raw trace/XML/robot pose is in `evaluation/trials`.
`verification_dt/comparison.json` contains the refinement results.
`EVALUATION_DESIGN.md` records the hypotheses and comparison rules.

The manuscript includes these results in Section V-E and Figure 6, with the
quality study in Section VI. Its preceding source/PDF are archived in
`../RoboSoft_EIT_VMC_Draft/revisions/before_ball_followup_20261022/` relative to
the parent simulation directory. The paper packages summaries under
`evidence/ball_followup/`; this directory retains full reproducibility evidence.

