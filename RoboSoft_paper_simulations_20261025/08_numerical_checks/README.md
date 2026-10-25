# Numerical and implementation checks

Section VI and supporting verification

`followup_half_timestep/` contains four refined traces. `followup_timestep_comparison.json` maps them to the original four 1 ms runs in the progress/attachment folders. All retain completion; final-error changes are at most approximately 0.00493 mm.

`original_timestep/` contains four earlier checks: joystick and ball, each at 1 ms and 0.5 ms. `original_task_checks.json` also reports point-Jacobian equivalence, equivalent contact-frame torque, transmission power consistency and FEM mesh sensitivity. `earlier_support_checks.json` preserves earlier support-model verification.

`followup_baseline_equivalence.json` records a zero difference between original and extended trial implementations for one nominal check. It is an individual check, not proof that every possible configuration is equivalent.

Code: `../joystick_ball_sim/src/verify.py`, `../ur3e_sim/src/verify.py` and frozen follow-up trial source. Use the portable runner to rerun the eight indexed timestep cases; reading the saved verification reports needs no simulation.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 08` to rerun all indexed cases here, or add `--subset NAME --limit 1` for a short check. Original evidence is preserved.
