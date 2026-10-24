# Offline validation — 24 October 2026

No robot or EIT device was connected or reconfigured for this update.

- Extracted local collection/training/prediction modules, serial reader, RBF/MLP code and both engraved pad meshes. The original collector and session/model files were preserved.
- Ran 54 toolkit tests: 47 passed and 7 GUI workflow tests were skipped because no authorized graphical display was available. Tests cover 8/16-electrode scans, RBF/MLP fitting and persistence, protocol/gain handling, raw clipping/zero preservation, gain-dependent range estimates and saved-session review.
- After the zero-baseline diagnostic fix, reran the seven focused range/plot tests successfully. A missing baseline channel no longer hides valid relative changes on the remaining channels.
- Copied the toolkit into an isolated temporary folder and verified pad rendering, serial-protocol and diagnostic tests without the original project paths.
- Rendered and visually inspected a synthetic signal plot with a zero channel, a low reported clipped amplitude and a near-ceiling channel. The clipping status remains visible independently of the small raw value.
- Checked Python syntax and Git filtering: code, local meshes and models selected into models/ are included; raw sessions, snapshots and caches are ignored.

The full GUI window and actual sensor rate, analogue scaling on firmware v1.2.1, signal quality, position accuracy and physical contact repeatability still need checking on the sensor computer. The voltage ceiling is a sinusoidal estimate from supplied v1.1.1 source and active RX gains, not a direct measurement of ADC waveform peaks. Synthetic tests establish software behavior, not pad accuracy.
