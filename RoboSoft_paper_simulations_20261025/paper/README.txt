Adapting to Contact: Tactile Configuration of Virtual Mechanics

Main document: RoboSoft_EIT_Informed_VMC_Draft.pdf (6 pages)
Editable source: manuscript.tex


Revision: 24 October 2026, solid Figure 3 bars
Removed bar hatch patterns and outlines from Figure 3; bars now use solid
categorical colours. Individual-trial markers, labels and numerical values
are retained. The caption was updated accordingly. Regenerated PDF/PNG/SVG
and rebuilt the paper. Previous files are in
revisions/before_solid_figure3_bars_20261024/.

Revision: 23 October 2026, figure color and mechanical-link review
All four current paper figures now use consistent categorical colors,
dark diagram annotations and redundant labels, symbols or hatch patterns.
Quantitative heatmaps use cividis_r and ordered Blues brightness with
contrast-selected black/white numbers. The framework has an explicit
contact label. The source illustration and saved simulation states are
unchanged. Shared definitions: figure_style.py. The framework can be
regenerated with make_figures.py --framework-only.
Figure 2b and its standalone preview now use solid bidirectional mechanical
links, without a separate dashed reaction path or label. Controller
equations and numerical results are unchanged.
Validation: all four figures inspected in normal, grayscale and simulated
protanopia/deuteranopia/tritanopia views. Main diagram text contrast on
white exceeds 4.5:1; heatmap text exceeds 7.5:1. This is a visual figure
review, not complete PDF accessibility certification. Audit:
evidence/figure_color_audit.json; previews: figure_review/.
Six-page PDF rebuilt; no overfull boxes or unresolved references. Previous
files: revisions/before_figure_color_accessibility_20261023/ and
revisions/before_bidirectional_ball_links_20261023/.

Revision: 23 October 2026, requested section removal
Removed the standalone AI assistance section from manuscript.tex and the
compiled PDF. The document remains six pages. Previous files are archived
in revisions/before_remove_ai_assistance_20261023/.
Validation: PDF rebuilt; section absent; no overfull boxes or unresolved references.

Revision: 23 October 2026, sensor-quality results figure
Figure 4 now displays all 36 frozen bias/delay trials: mean final ball
position error and all-criteria completion counts for three seeds per cell.
The text notes the condition-dependent delay effect at +5 mm bias. The
figure uses existing synthetic evidence; no simulations were rerun.
Regenerate it with make_sensor_quality_results.py. Per-cell trial IDs,
values and source hash are in evidence/sensor_quality_figure_provenance.json.
The obsolete manual reference-column break was removed to retain six pages.
Validation: all 12 cells checked against the 36 raw records; six-page PDF
rebuilt and final page visually inspected; no overfull boxes or unresolved
references. Previous files: revisions/before_sensor_quality_figure_20261023/.

Revision: 23 October 2026, joystick angle definition
Figure 2a now measures the actual joystick angle theta from a dashed
upright reference about the physical pivot, with an angular arrow ending
on the current shaft. The figure caption defines this convention.
Validation: regenerated vector figure and rebuilt six-page PDF; no overfull
boxes or unresolved references. Previous version:
revisions/before_joystick_angle_arrow_20261023/.

Revision: 23 October 2026, clearer task mechanism diagram
The ball panel places a dashed desired ball at the fixed goal s_g, with a
dotted guide to the goal spring and a distinct dashed coupling-reaction
return through the 2:1 transmission. The virtual components are arranged
below the physical contact. A standalone preview is available as
figures/ball_roll_mechanism.pdf (also PNG and SVG).
Joystick diagrams use theta_d for the commanded angle and theta for the
actual angle; the text explains that the arc and palm orientation share
the commanded angle. The common figure equation uses v_ref for commanded
reference velocity. No controller or simulation results were changed.
Validation: rebuilt six-page PDF, no overfull boxes or unresolved references.
Previous files: revisions/before_goal_ball_diagram_20261023/.

Latest revision: 23 October 2026, attachment terminology
The manuscript and mechanism figures now say that EIT defines the VMC
attachment at contact. The joystick attachment remains fixed in the palm
frame during manipulation; ball attachment updates retain their previous
meaning. Simulation conditions and numerical results are unchanged.
Validation: PDF rebuilt with Tectonic; six pages including references.
Updated figure inspected; no overfull boxes or unresolved references.
Previous source, figures and PDF are archived in:
revisions/before_attachment_wording_20261023/.

Latest revision: 22 October 2026, six-page task-focused manuscript
The current paper has six pages including references, using the standard
IEEE conference class, 10 pt font and US letter pages. The standalone
"Isolating the Effect of Contact Attachment" section, its two figures and
results table are removed from the main paper. The mechanical explanation
remains in Section III; shared robot, synthetic EIT and decoder details are
in Section IV. Section V contains joystick and ball results, Section VI
contains completed sensing-quality tests, and Section VII combines limits
and the next validation priorities. Longer future-study plans are condensed.

Current figures: 1 framework (page 2), 2 mechanisms and simulated setups
(page 3), 3 combined task results (page 5). Historical notes below retain
the figure and section numbers of their respective revisions.

The discussion explicitly identifies stronger orientation control and
force comparison at matched accuracy/completion as outstanding tests.
Known joystick geometry, conditional scoring denominators, unsuccessful
conditions, nominal no-benefit results and simulation limitations remain
visible. These edits do not establish a new controller performance result.

Detailed support experiments remain in:
../ur3e_sim/report/UR3e_EIT_VMC_Simulation_Report.pdf
Full task and follow-up evidence remains in:
../joystick_ball_sim/report/Joystick_Ball_EIT_VMC_Simulation_Report.pdf
../joystick_ball_sim/followup_20261022/report/Ball_Rolling_Followup_Report.pdf
The previous ten-page draft and summary are archived in:
revisions/before_six_page_revision_20261022/

Validation: rebuilt twice; all six pages visually inspected; references
resolve and there are no overfull boxes. Figure inputs and raw evidence
are unchanged; no simulations were run for this revision.



Earlier revision: 22 October 2026, clearer task labels
Figure 5 labels now say "Final ball position error", "Robot motion estimate",
"Maximum palm tilt", and "Movement without contact". Initial and updated
EIT contact labels replace the abbreviated attachment names.
Section V-E defines final error as the mean absolute target distance over
7-8 s after engagement, and explains the robot-motion estimate: initial
camera position plus half the palm travel, assuming rolling without slip.
Previous files: revisions/before_plain_task_labels_20261022/.

Earlier revision: 22 October 2026, merged task evidence
Figures 5 and 6 are consolidated into Figure 5 (page 8):
(a) joystick attachment, (b) ball progress after a push,
(c) ball attachment during migration, (d) contact-loss recovery.
The older nominal-roll panel and stop-restart dynamics panel are removed
from the main results figure. Nominal no-benefit evidence is summarized in
Section V-D, and the exploratory overshoot/lag tradeoff is qualified in the
Discussion. Full simulation evidence and reports are retained.

Section V-E explains the disturbance and migration experiments. The abstract,
conclusion, hardware protocol and five-bullet summary now emphasize these
sensing benefits rather than carriage superiority. The joystick table no
longer repeats the older nominal ball comparison.

Regenerate the combined figure with make_main_task_results.py. It reads only
packaged evidence and does not run simulations. Per-panel trial identifiers,
means and source hashes are in evidence/main_task_figure_provenance.json.
make_task_results.py and the follow-up project's paper_figure.py regenerate
historical figure assets, not the current combined figure.

Validation: PDF rebuilt twice; all figure references resolve, no overfull
boxes or LaTeX warnings; pages 7 and 8 visually inspected. No simulation
runs or input evidence were changed. Previous manuscript, PDF, figures,
summary, README and manifest: revisions/before_merged_task_results_20261022/.

Earlier revision notes follow.

Previous revision: 22 October 2026, executed ball follow-up
329 frozen evaluation attempts after 68 exploratory trials test progress feedback,
attachment migration, reference dynamics, and progress bias/delay/dropout.
Section V-E and new Figure 6 report the matched findings; Section VI adds the
completed quality grid. EIT progress feedback reduces forward-push late error
from 2.73 to 0.41 mm (8/8 paired improvements); both conditions pass 8/8.
Updated attachment reduces mean peak pitch from 4.01 to 1.92 degrees versus
an attachment defined at initial contact at low orientation stiffness. Carriage dynamics reduce stop–restart
overshoot but increase integrated error. The paper retains severe-push/edge
failures and the benefit of EIT feedback with the prescribed-reference baseline.

Full report and raw evidence:
../joystick_ball_sim/followup_20261022/report/Ball_Rolling_Followup_Report.pdf
../joystick_ball_sim/followup_20261022/evaluation/
Packaged paper evidence: evidence/ball_followup/.
Previous paper: revisions/before_ball_followup_20261022/.
The ten-page PDF was rebuilt and checked for references, overflow, and layout.
The following entries describe earlier revisions; their planned-study statements
are superseded where the new follow-up now supplies executed results.


Current revision: 22 October 2026, contribution and evaluation structure
The paper centres on EIT configuration of virtual components and the sensing
quality required by the task. Section IV now isolates contact attachment with
shorter loading/reorientation text. Section V distinguishes the joystick attachment
result, ball reference dynamics, and contact-loss recovery. The ball discussion
explains coupling reaction and identifies the controller-dependent stopping rule
as a limit on the endpoint-error result. Figure 4 retains the palm-above layout.

Section VI separates completed bias/rate/noise evidence from a proposed study of
localization, delay, migration, curved-object calibration, and dropout. Table III
maps those tests to calibration, training, acquisition, and control decisions.
Matched resistance/stop--restart tests, a carriage-reaction ablation, and physical
validation are explicitly planned, with no new experimental results claimed.
The hardware protocol prioritizes the matched joystick comparison. Related work
now includes Larby and Forni's dynamic virtual-mechanism formulation, verified
against arXiv:2411.06627 version 3 on 22 October 2026. The five-bullet summary
matches the revised scope.

Validation: the nine-page PDF was rebuilt with pdflatex, extracted to text, and
rendered for layout inspection. No unresolved references or overfull boxes remain.
Existing numerical evidence and figure assets match the pre-revision manifest.
The previous manuscript/PDF/summary/README/manifest are preserved in
revisions/before_contribution_revision_20261022/.

Revision history follows.

Revision: 21 October 2026, task simulation results
Section IV retains the controlled support experiments. Section V adds executed
joystick, ball, and interruption simulations to inform expected hardware results.
All physical task validation remains planned. Abstract, discussion, conclusion,
framework caption, and hardware protocol now reflect this evidence status.

Primary task presentation
Joystick: spring--damper controller with a prescribed moving arc reference,
normal preload, orientation control, and shared touch detection. Centre attachment
passes 9/27 and EIT attachment 27/27. This is not a pure spring. Centre failures
include 11 rim-guard and 7 horizon terminations under assumed housing geometry.
Ball: virtual carriage. Centre and EIT attachments both pass 27/27 short and 6/6
long rolls. This does not demonstrate a spatial adaptation benefit. Both receive
the same tactile progress feedback. The task provides an integration/feasibility
case and an explicit limitation of the adaptation claim.
Unexpected loss: EIT loss-aware recovery passes 6/6 versus 1/6 scheduled trials,
with reduced disconnected travel but higher recovery load. This supports event
handling; it does not isolate an advantage from task-model topology switching.
The unfavorable virtual-lever results are retained in Section V-B.

Evidence provenance
../joystick_ball_sim/results contains the original frozen 540-trial campaign.
../joystick_ball_sim/paper_supplement contains 27 added point-controller centre
trials, their protocol, saved states, metrics, and audit. This matched comparison
was added after inspection of the original results, with no gain/criterion changes;
its seeds repeat the original J2 trials and are not a new independent holdout.
evidence/task_simulations packages original metrics, supplement metrics/protocol,
and the selected comparison CSV and summary. Older evidence files support Section IV.
The original campaign also froze the then-current manuscript as a source input.
That exact pre-edit manuscript and its PDF/README/manifest are preserved under
revisions/before_task_results_20261021/. The current draft is a later revision;
the original campaign protocol and results were not rewritten.
Full task traces/code remain in the adjacent simulation project. Figure 4 pairs mechanism schematics with saved-state MuJoCo renders; Figure 5
is generated from saved numerical data.

Build
Run pdflatex -interaction=nonstopmode -halt-on-error
-jobname=RoboSoft_EIT_Informed_VMC_Draft manuscript.tex twice (one command line).
make_figures.py regenerates the original framework/support figures.
make_task_results.py regenerates task evidence and figures from the adjacent
simulation project. It requires NumPy, pandas, and Matplotlib. The session used:
MPLCONFIGDIR=/tmp/mpl_eit PYTHONPATH=/tmp/eit_task_deps
/home/yi/miniconda3/envs/mujoco312/bin/python make_task_results.py
Simulation runtime details are recorded in ../joystick_ball_sim/README.md.

Evidence limits and hardware priorities
Synthetic EIT uses an existing jointly position/force-trained Extra Trees decoder;
its force output does not enter task control. Electrical constitutive behavior,
noise, material properties, dimensions, recessed housing and contact compliance
are assumptions. Ideal torque control and exact bias compensation are simulated.
No measured hardware success, sensor latency, force accuracy, full arm clearance,
or generic advantage over equivalent contact-frame impedance is established.
Measure curved-object sensing, bracket geometry, friction/indentation, available
robot control interface, and independent task/loading/rotation before physical tests.
The fixed timer comparison is a supporting diagnostic. Joystick centre versus EIT
with the same detector is the primary spatial comparison. Ball rolling currently
supplies no demonstrated attachment benefit; a sensing benefit needs a separate
matched feedback comparison. A conventional force detector is a useful hardware
reference if independently available.

Source notes
IEEEtran v1.8b is bundled with its original licensing text. Existing references
were checked during the preceding revisions. The current revision adds the
Larby and Forni reference noted above.
AI-generated human-palm motivation remains in Figure 1. The earlier generated
task concept illustration is retained as an asset but is not in the revised PDF.
The companion 19-page task report remains a separate document.

Revision: 22 October 2026, VMC structure figures
Figure 2 shows the support loading/reorientation spring and zero-velocity damper.
The task structure diagrams were subsequently integrated into Figure 4 (see below).
Regenerate the PDF/PNG/SVG diagrams with draw_task_vmc.py (Matplotlib/NumPy).
The pre-figure manuscript and PDF are preserved in revisions/before_vmc_figures_20261022/.

Revision: 22 October 2026, improved integrated Figure 4
Figure 4 now combines the joystick arc guide and ball carriage with corresponding
MuJoCo contact views. The carriage drawing explicitly shows goal spring, virtual
mass, damping, 2:1 transmission, and coupling reaction. The joystick image comes
from the evaluated point-controller trial 131; ball image from carriage trial 237.
The old separate task-structure float is merged into Figure 4. Regenerate with
draw_figure4.py using the MuJoCo environment and settings above; frame provenance
is in evidence/task_simulations/figure4_provenance.json. PDF, PNG and SVG outputs
are figures/task_setup_mechanisms.*. Previous files are preserved in
revisions/before_integrated_figure4_20261022/.

Revision: 22 October 2026, Figure 4 orientation
Both task schematics now place the palm above the contacting cap or ball.
Virtual components connect from the selected underside attachment and lie below
and beside the physical object. The diagram matches the orientation of the
MuJoCo views. Previous source/PDF are in revisions/before_palm_above_20261022/.
