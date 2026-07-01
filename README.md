# FabScan v0.5.8 - Bounded Multi-Step Edge Follow

v0.5.8 adds a bounded `Follow N` mode to the Camera Calibration Lite window.

This is still not free-running continuous following. It simply repeats the already-tested single-step follow logic for a user-selected number of steps. Each step re-detects the line/edge, checks confidence, calculates a bounded tangent/correction move, waits for LinuxCNC to settle, and stops if anything looks wrong.

## What changed

- Added `Count` to the Single-Step Follow panel.
- Added `Follow N` button.
- Added a stop-request flag so `STOP Move` also stops a running Follow N sequence.
- Saved/restored follow count in settings.
- No Z motion, no torch, no MDI, and no continuous chasing.

## Safety behavior

Follow N stops if:

- LinuxCNC is not ON / IDLE / MANUAL.
- X/Y/Z are not homed.
- The line/edge is lost.
- Detection confidence drops below Min conf.
- A jog fails or does not settle.
- STOP Move is pressed.
- The requested count is completed.

The count is clamped to 1–50 steps.

## Suggested first test

Start conservative:

```text
Step: 0.025 or 0.050
Max correct: 0.025 or 0.050
Min conf: 45
Count: 3
Feed: 5
Capture after move: off for the first test
```

Workflow:

```text
1. Open Camera Calibrate.
2. Make sure calibration is valid.
3. Put a clean line/edge under the camera.
4. Use Find Line / Edge and confirm the overlay looks right.
5. Enable follow.
6. Click Follow Step once.
7. If direction is correct, set Count to 3 and click Follow N.
8. Press STOP Move if anything looks wrong.
```

## Install

Copy these files over an existing FabScan v0.5.7 checkout:

```bash
cp fabscan/app.py ~/projects/FabScan/fabscan/app.py
cp fabscan/camera_calibration.py ~/projects/FabScan/fabscan/camera_calibration.py
cp fabscan/settings.py ~/projects/FabScan/fabscan/settings.py
cp README_v0.5.8.md ~/projects/FabScan/README.md
```

Then run:

```bash
cd ~/projects/FabScan
source .venv/bin/activate
python3 -m py_compile fabscan/*.py fabscan.py
python3 fabscan.py
```
