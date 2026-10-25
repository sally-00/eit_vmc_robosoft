# Sensor quality and task performance

Section VI; Figure 4

Figure 4 shows all 36 trials in `bias_delay_sweep/`: three biases (−5/0/+5 mm) × four added delays (0/100/250/500 ms) × three seeds. Centre attachment and contact detection remain common; the feedback position is transformed using the sampling pose before it is delayed.

With zero added bias all three trials pass at every tested delay. At zero delay, either sign of 5 mm bias leaves 1/3 passing, with approximately 5.1 mm mean final error. Delay effects are condition-dependent; the positive-bias 500 ms condition passes 3/3. This slow task does not establish a general latency requirement.

`position_update_dropout/` retains nine trials with 0.5/1/2 s progress-update dropout; all complete after updates resume, with contact events still available. `high_electrical_noise_failures/` retains six earlier high-noise trials (three joystick, three ball); all fail. Position dropout is not complete sensing loss.

Code: follow-up `trial_followup.py` for position bias/delay/dropout; original `trial.py` and `sensing.py` for electrical noise. Fields `common_success` and `late_error_mm` score follow-up trials; original noise trials use `success`.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 06` to rerun all indexed cases here, or add `--subset NAME --limit 1` for a short check. Original evidence is preserved.
