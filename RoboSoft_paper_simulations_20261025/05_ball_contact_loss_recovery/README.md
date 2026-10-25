# Ball contact-loss recovery

Section V-D; Figure 3(d)

Figure 3(d) uses the 12 trials in `scheduled_vs_tactile_recovery/`: six scheduled-EIT versus six event-EIT trials, paired at ±10 mm and three seeds. During a requested 60 mm roll, the table withdraws 20 mm at 2.50 s, with the specified 0.15 s lowering, 0.40 s hold and 1.20 s return. The controller receives no fixture timing flag.

Tactile recovery passes 6/6 versus scheduled control 1/6. Mean palm motion while detached falls from 33.62 to 4.02 mm, while mean peak load rises from 3.48 to 6.20 N. This is a recovery tradeoff, not evidence of reduced load. Mean detection response is 91 ms; sustained stopping is approximately 405 ms. The electrical signal returns to baseline immediately on detachment in this model.

Code: `../joystick_ball_sim/src/trial.py`. The original campaign's other B4 policies are retained in aggregate provenance, but these are the two conditions presented in the paper.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 05` to rerun all indexed cases here, or add `--subset NAME --limit 1` for a short check. Original evidence is preserved.
