# Ball progress correction after a push

Section V-B; Figure 3(b)

Main result: Figure 3(b) selects `case=forward_4N` from `progress_feedback/`. The main 24 trials compare robot-motion, EIT and true-contact progress with shared centre attachment and contact detection, two offsets and four paired seeds. A 4 N forward force acts for 0.35 s beginning 2 s after engagement during a 20 mm/8 s roll.

Mean final-second ball-position errors are 2.73, 0.41 and 0.05 mm respectively; all pass 8/8. EIT improves all eight paired errors. The 120-trial subset also retains nominal, reverse/other pulse and severe-force cases; the 6 N force terminates all 24 attempts. Early terminations receive no final-error score.

`prescribed_reference_control/` has 16 trials, showing that sensing also helps without a reaction-driven carriage: robot-motion error 2.23 mm versus EIT 0.46 mm, with 8/8 passing each. Thus improvement is not unique to carriage dynamics.

Code: `../joystick_ball_sim/followup_20261022/src/trial_followup.py`. Inspect `case` and `progress_source` when selecting plotted results. Simulator truth is used only by the oracle comparator, sensor synthesis, evaluation and common guards.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 03` to rerun all indexed cases here, or add `--subset NAME --limit 1` for a short check. Original evidence is preserved.
