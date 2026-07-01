# FabScan v0.5.7 - Calibration Window Layout Cleanup

This release cleans up the Camera Calibration Lite window layout after the first single-step edge-following test.

## What changed

The motion and camera behavior is unchanged from v0.5.6. This is a GUI/layout patch only.

Changes:

- Shortened the top camera/setup area.
- Moved Dot / Camera / Calibration / Line status to the left side of the live preview.
- Moved Dot Center Jog controls to the right side of the live preview.
- Moved Single-Step Follow controls to the right side of the live preview.
- Kept Line / Edge Preview controls near the status area.
- Reduced the starting window height so the lower preview area is less likely to be cut off on the plasma PC display.
- Increased the live preview box slightly while keeping it fixed-size so the window does not resize while video updates.

## Install

Copy the drop-in files over your existing project:

```bash
cd ~/projects/FabScan

cp /path/to/unzipped/FabScan_v057_cal_layout_dropin/fabscan/app.py fabscan/app.py
cp /path/to/unzipped/FabScan_v057_cal_layout_dropin/fabscan/camera_calibration.py fabscan/camera_calibration.py
cp /path/to/unzipped/FabScan_v057_cal_layout_dropin/README_v0.5.7.md README.md
```

Then test:

```bash
source .venv/bin/activate
python3 -m py_compile fabscan/*.py fabscan.py
python3 fabscan.py
```

## Test notes

Open **Camera Calibrate** and verify:

- The top controls no longer burn up so much vertical space.
- The Dot / Camera / Calibration info is visible to the left of the preview.
- The jog and follow controls are on the right of the preview.
- The bottom of the live preview is no longer cut off.
- Dot calibration, Center Dot, Find Line / Edge, and Follow Step behave the same as v0.5.6.

## Commit

```bash
git status
git add fabscan/app.py fabscan/camera_calibration.py README.md
git commit -m "Clean up camera calibration window layout"
git push
```
