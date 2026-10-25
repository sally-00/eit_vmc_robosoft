# Ball moving contact attachment

Section V-C; Figure 3(c)

Figure 3(c) compares centre, initial EIT contact, updated EIT contact and updated true contact. All receive the same EIT progress feedback. `attachment_gain_sweep/` has 96 trials: +10/+25 mm offsets × orientation stiffness 0.3/1/3 Nm/rad × four seeds × four attachment choices.

The figure shows the 32 trials at 0.3 Nm/rad. Mean peak palm tilts are 2.98°, 4.01°, 1.92°, 1.92° respectively, all completing 8/8. This supports alignment improvement at finite rotational compliance, not a completion or lowest-load advantage. The initial/updated difference narrows at higher stiffness.

`negative_edge_failures/` retains the 16 additional −25 mm edge trials; every one terminates. The geometry limits of the continuous stroke are part of the evidence.

Code: `../joystick_ball_sim/followup_20261022/src/trial_followup.py`. Historical configuration value `attachment=latched` denotes the initial EIT attachment retained in palm coordinates; the paper calls it initial EIT contact. Updated attachments are filtered/bounded and rebase anchors.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 04` to rerun all indexed cases here, or add `--subset NAME --limit 1` for a short check. Original evidence is preserved.
