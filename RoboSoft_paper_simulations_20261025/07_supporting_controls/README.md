# Supporting nominal roll and dynamics controls

Section V-C and Discussion

These controls are discussed in the paper text rather than plotted in its main result panels.

`nominal_ball_attachment_control/`: 54 earlier 25 mm rolling trials, 27 centre and 27 updated-EIT attachments, both with EIT progress feedback. Both pass 27/27; mean camera-target errors are 0.18 and 0.32 mm respectively. This nominal comparison shows no attachment benefit.

`stop_restart_and_resistance_dynamics/`: 36 frozen follow-up trials spanning prescribed-reference, carriage and carriage-without-reaction controllers. The paper notes that the carriage reduces overshoot in exploratory stop–restart behavior but increases integrated error; no matched-responsiveness advantage is established. These evaluation trials remain distinct from the excluded 68 pilot trials.

Code: original `trial.py` for nominal rolls; follow-up `trial_followup.py` for dynamics. `reaction_scale=0` is an intentional ablation, not the main carriage controller.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 07` to rerun all indexed cases here, or add `--subset NAME --limit 1` for a short check. Original evidence is preserved.
