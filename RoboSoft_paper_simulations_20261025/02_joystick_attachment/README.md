# Joystick attachment accuracy and load

Section V-A; Figure 3(a)

Main result: Figure 3(a), 54 trials in `main_attachment_comparison/`: 27 centre and 27 EIT attachments. Offsets −25/0/+25 mm, clearances 2/6/14 mm and three paired sensor seeds; +12° then −12° commands. The attachment is defined at contact and retained in palm coordinates. Both conditions share detection, geometry, gains and prescribed-reference generator.

EIT passes 27/27 versus centre 9/27. Mean peak force is 4.19 versus 5.61 N. Mean angle error is 0.32° for EIT; centre has 2.84° among only 18/27 scored trials. Missing scoring windows remain missing; every attempt remains in completion denominators. These are not force/error comparisons at matched completion.

`reaction_driven_lever_control/` contains 27 reaction-driven EIT lever trials: both lever and prescribed arc pass 27/27, with 1.29° versus 0.32° mean angle error. `low_return_stiffness_control/` contains 18 trials at 0.05 Nm/rad: lever 0/9 versus prescribed reference 9/9. They limit any claim of a benefit from extra virtual joystick dynamics.

Code: `../joystick_ball_sim/src/trial.py`, `scene.py`, `sensing.py`. `mechanism=point` means the prescribed arc/reference generator; the contact spring–damper and Jacobian-transpose torque mapping are still applied.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 02` to rerun all indexed cases here, or add `--subset NAME --limit 1` for a short check. Original evidence is preserved.
