# Download-bundle validation — 25 October 2026

No physical robot or EIT device was accessed. Original workspace simulation projects and results were left unchanged.

- Bundled 508 indexed saved trajectories: the paper's main comparisons and described controls/failures, all 329 frozen ball follow-up trials, and eight timestep-check traces. Synthetic sensor data include 480 training, 80 validation, 100 held-out test and 100 material-shift episodes and saved predictions.
- Checked every indexed NPZ configuration against its schedule, trace dimensions, increasing timestamps and finite robot/object states. True contact X is allowed to be undefined outside contact in the follow-up traces, matching the original audit.
- Verified all frozen simulator/runtime source and decoder hashes. The campaign's historical manuscript hash is identified separately because the current paper has been revised since that campaign.
- Recomputed Figure 3 completion counts/means and all 12 Figure 4 cells from the packaged analyzed metrics; they match the copied paper evidence. `SAVED_RESULTS_SUMMARY.json` stores the full summary.
- Compiled all bundled Python source for syntax checks. The portable list and saved-result summary commands work without simulation dependencies.
- Ran one original joystick case (`original_0005`) and one follow-up ball case (`followup_0000`) from the new folder with the original archived functions/configurations, writing only to temporary directories. Both retained success and matched the saved trace arrays exactly (maximum finite difference 0.0). Details are in `RERUN_SMOKE_CHECKS.json`. These are two smoke checks, not a rerun of all 508 trajectories.
- Temporary rerun environment: Python 3.12, NumPy 2.3.4, SciPy 1.18.1, MuJoCo 3.5.0, scikit-learn 1.8.0, joblib 1.6.0, threadpoolctl 3.7.0 and PyYAML 6.0.3. Original environment/requirements records are preserved in `provenance/` and shared source folders. Bitwise equality on other environments is not guaranteed.
- The ZIP is checked for member CRC errors and file-by-file SHA256 agreement with `SHA256_MANIFEST.json`. The adjacent `.zip.sha256` checks the downloaded ZIP itself.

The synthetic EIT training computation, all legacy campaign launchers, paper PDF compilation and OpenGL figure regeneration were not rerun during packaging. Existing training arrays, predictions, full figure sources and current six-page PDF are included. The portable trial wrapper preserves the archived data and uses separate rerun directories.
