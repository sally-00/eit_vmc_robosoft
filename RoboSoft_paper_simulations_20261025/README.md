# RoboSoft paper simulation archive

Prepared 25 October 2026 from the current six-page draft, **Adapting to Contact: Tactile Configuration of Virtual Mechanics**. All results here are synthetic. This archive has no physical robot-control or EIT-device dependency.

The numbered folders follow the order in which experiments appear in the paper. They include the main comparisons, failures and controls described in the text, not only the successful plotted examples. There are **508 indexed saved simulation trajectories**, plus synthetic EIT training/validation/test datasets, predictions, the restored decoder, frozen controller/physics/sensing code, upstream robot parameter files, source-hash records and figure-generation code.

## Read the results first

Open `paper/RoboSoft_EIT_Informed_VMC_Draft.pdf`, then browse these folders:

| Order | Folder | Paper location | Saved trajectories |
|---|---|---|---:|
| 01 | `01_synthetic_eit_validation/` | Section IV-B: synthetic sensor and decoder | Datasets: 480 training, 80 validation, 100 test and 100 shifted episodes |
| 02 | `02_joystick_attachment/` | Section V-A; Figure 3(a) | 99: main 54, lever control 27, low-stiffness control 18 |
| 03 | `03_ball_progress_after_push/` | Section V-B; Figure 3(b) | 136: feedback conditions 120, prescribed-reference controls 16 |
| 04 | `04_ball_moving_attachment/` | Section V-C; Figure 3(c) | 112: attachment/gain sweep 96, negative-edge failures 16 |
| 05 | `05_ball_contact_loss_recovery/` | Section V-D; Figure 3(d) | 12 paired trials |
| 06 | `06_sensor_quality/` | Section VI; Figure 4 | 51: bias/delay 36, position-update dropout 9, high-noise failures 6 |
| 07 | `07_supporting_controls/` | Nominal-roll paragraph and Discussion | 90: nominal attachment control 54, dynamics controls 36 |
| 08 | `08_numerical_checks/` | Numerical verification discussed in Section VI | 8 timestep-check trajectories, plus Jacobian/power/mesh/equivalence reports |

Each experiment has a README. Each trajectory experiment also has `trial_index.json`; each subset has the original NPZ trajectories and filtered `analyzed_metrics.csv`. `EXPERIMENT_INDEX.json` is the machine-readable table of contents. `FILE_ORIGINS.json` maps copied files to their workspace source and SHA256. The NPZ files retain their original configuration, trace, XML, robot/object positions, events and worker metrics. Unique filenames distinguish overlapping trial IDs from different campaigns.

The restored synthetic decoder is provided at `joystick_ball_sim/results/eit_estimator_restored.joblib`. It matches the original model manifest exactly. The truncated legacy checkpoint is excluded; a byte-identical copy of the restored model is supplied at `ur3e_sim/results/eit_estimator.joblib` so the older sensor/support source can load a valid checkpoint. The original restoration audit is included.

## Verify and summarize without rerunning simulations

From this folder:

```bash
python3 verify_bundle.py
python3 summarize_results.py > paper_results_summary.json
```

These two commands use the Python standard library. To additionally inspect every saved trajectory's configuration, shape, timestamps and numerical state, install NumPy and run:

```bash
python3 verify_bundle.py --traces
```

The summary reproduces the paper's sensor metrics, Figure 3 means/counts and Figure 4 cells directly from saved evidence. It retains failed attempts and does not invent errors for early-terminated trials.

## Install simulation dependencies

Python 3.12 was used for the archived studies. On the download computer:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
```

The top-level requirements follow the task campaign, including MuJoCo 3.5.0 and scikit-learn 1.8.0. Both original requirements files and runtime-version records remain in their source/provenance folders. Some archived imported package versions differed from distribution metadata; the records document this. Installing the recorded dependencies does not guarantee bit-for-bit equality across operating systems and numerical libraries. Compare results with the archived tolerances and saved numerical checks.

The EIT decoder was trained synthetically with position and force labels, but its force output does not enter task control. It is unrelated to the physical-pad model in the separate hardware EIT toolkit.

## Rerun in paper order

Use the portable wrapper instead of the legacy full-campaign launchers. The wrapper uses the unchanged archived trial functions and configurations. It writes to a new timestamped `reruns/` directory and rejects destinations inside the evidence/source folders. Listing and saved-result inspection do not run physics.

```bash
python run_experiment.py --list
```

A short reproduction check is one main joystick trial:

```bash
python run_experiment.py --experiment 02 --subset main_attachment_comparison --limit 1
```

Then the paper's main experiments, in order:

```bash
python run_experiment.py --experiment 02 --subset main_attachment_comparison
python run_experiment.py --experiment 03 --subset progress_feedback
python run_experiment.py --experiment 04 --subset attachment_gain_sweep
python run_experiment.py --experiment 05
python run_experiment.py --experiment 06 --subset bias_delay_sweep
```

The progress-feedback block includes all frozen feedback cases; Figure 3(b) specifically selects `case=forward_4N`. The attachment block includes all three orientation gains; Figure 3(c) specifically selects 0.3 Nm/rad. To reproduce all archived controls/failures in a section, omit `--subset`:

```bash
python run_experiment.py --experiment 02
python run_experiment.py --experiment 03
python run_experiment.py --experiment 04
python run_experiment.py --experiment 06
python run_experiment.py --experiment 07
python run_experiment.py --experiment 08
```

To regenerate the synthetic sensor datasets and train its decoder from scratch:

```bash
python run_experiment.py --experiment 01
```

That command creates a new output directory, with newly generated `results/eit_*.npz`, decoder and metrics; it never replaces the archived model used for the paper. It is a larger computation than reading the saved predictions. The wrapper runs selected episodes sequentially. Each trial retains its finite horizon and original stop guards.

Worker reruns save raw metrics. The original analysis code in `joystick_ball_sim/src/analyze.py` and `joystick_ball_sim/followup_20261022/src/analyze_followup.py` documents derived scoring and missing-window handling. Those original scripts expect full legacy campaign layouts and historical source paths. They are preserved as evidence; their direct invocation is not the portable rerun route. In particular, raw `success` and follow-up `common_success` are distinct fields. Use the archived analyzed CSVs for the published comparisons.

## Regenerate the current result figures

From the archive root, after installing requirements:

```bash
python paper/make_main_task_results.py
python paper/make_sensor_quality_results.py
```

They regenerate Figure 3 and Figure 4 from the copied paper evidence without rerunning physics. Figure 3 uses solid coloured bars. Figure 2's schematics and saved-state renders can be regenerated with:

```bash
MUJOCO_GL=egl python paper/draw_figure4.py
```

That historical filename generates current **Figure 2**, not Figure 4. It requires a working rendering backend; the saved figure is already included. The two setup traces are supplied at the renderer's original expected paths. `draw_task_vmc.py` contains the corresponding standalone mechanism schematic source. `make_figures.py --framework-only` regenerates Figure 1 using the included illustration asset. The paper's LaTeX source and IEEE class are included; compiling the PDF requires a separate TeX installation.

Regenerating figures updates their files/provenance and therefore changes the bundle hashes. Verify the archive before regenerating, or keep an unchanged extracted copy.

## Provenance and scope

Shared source folders `joystick_ball_sim/` and `ur3e_sim/` retain their original relative layout so imports and robot parameter lookup work. Their original source code has not been edited for this archive. Every frozen simulator/runtime source hash is checked against the campaign records in `SOURCE_HASH_AUDIT.json`. The original campaign also froze a historical manuscript hash; the paper has since been edited, so that entry is explicitly marked as a historical document reference rather than an unchanged runtime source.

`provenance/` contains complete original schedules, aggregate metrics, audits, environment records and companion PDFs for context. Some of those schedules describe additional experiments outside the current paper. Only trajectories indexed in the numbered experiment folders are included. The 68 exploratory ball pilots are described in the archived design/report but are not mixed with the 329 frozen follow-up evaluation trials. All 329 frozen follow-up trajectories are included across sections 03, 04, 06 and 07.

The paper's joystick main comparison uses a prescribed arc with virtual contact spring–damper control. The ball main comparisons use the reaction-coupled carriage. Ground truth supplies synthetic electrical input, evaluation, oracle conditions and shared protective guards. Physical EIT calibration, real actuation, contact properties and hardware task performance are not established by this archive. Vendor robot assets retain their upstream license in `ur3e_sim/vendor/ur_description/LICENSE`.

See `VALIDATION.md` for checks performed while preparing this download.
