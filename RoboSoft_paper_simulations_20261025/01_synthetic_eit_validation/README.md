# Synthetic EIT decoder validation

Section IV-B; accuracy values in the text

The paper first validates the synthetic electrical model and its decoder. `results/` contains complete-episode training, validation, independent-mesh test and material-shift datasets, their labels and saved predictions. There are 480/80/100/100 episodes respectively; complete episodes/sessions stay together.

The selected 100-tree Extra Trees model uses minimum leaf size 1. Held-out location median is 0.907 mm and its 95th percentile is 4.349 mm. Held-out force MAE is 0.940 N, increasing to 1.451 N under material shift. These are synthetic performance figures, not physical EIT accuracy. Force labels were used for training, but force predictions do not enter task control.

Code: `../ur3e_sim/src/eit.py` contains training/FEM; `../joystick_ball_sim/src/electrical.py` and `sensing.py` implement online simulated sensing. The restored model is in the shared simulator results directory. `estimator_restoration.json` records exact agreement with the original model manifest/predictions.

From the archive root, list the included subsets with `python run_experiment.py --list`. Use `python run_experiment.py --experiment 01` to regenerate synthetic datasets/training in a new output directory.
