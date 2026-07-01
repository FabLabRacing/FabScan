# FabScan v0.5.5 - Calibration Window Stability

This release fixes the Camera Calibration Lite window jumping/resizing while the live line/edge preview is running.

## What changed

- Camera Calibration Lite now starts with an explicit dialog size.
- The live preview now uses a fixed preview box.
- Preview images are scaled into that fixed box instead of resizing the whole dialog.
- Long status text is constrained so changing line/edge messages do not force the window wider.
- No motion behavior was changed.

## Install

```bash
cd ~/projects/FabScan

cp /path/to/unzipped/FabScan_v055_cal_window_stability_dropin/fabscan/app.py fabscan/app.py
cp /path/to/unzipped/FabScan_v055_cal_window_stability_dropin/fabscan/camera_calibration.py fabscan/camera_calibration.py
cp /path/to/unzipped/FabScan_v055_cal_window_stability_dropin/README_v0.5.5.md README.md
```

Then test:

```bash
source .venv/bin/activate
python3 -m py_compile fabscan/*.py fabscan.py
python3 fabscan.py
```

## Test

Open Camera Calibrate, enable Preview overlay, and move a line/edge under the camera. The dialog should stay stable while the preview and status update.

The camera calibration, dot centering, jog controls, and line/edge preview behavior are otherwise unchanged.
