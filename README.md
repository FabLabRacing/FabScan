# FabScan v0.5.6 - Single-Step Edge Follow

This release adds the first camera-driven line/edge follow move to Camera Calibration Lite.

The important guardrail is that this is **not continuous following**. One button press performs one bounded move:

1. Detect the current line/edge near the crosshair.
2. Convert the line offset to a limited X/Y correction using the saved camera calibration.
3. Convert the detected line angle to a small tangent move.
4. Send guarded X/Y incremental jogs through LinuxCNC MANUAL/JOG mode.
5. Stop and re-read the camera.

LinuxCNC remains the motion controller and the position ruler. FabScan only uses the camera to choose the next small correction.

## New controls

The Camera Calibration Lite window now has a **Single-Step Line / Edge Follow** panel:

- **Enable follow** - must be checked before Follow Step will move.
- **Step** - distance to move along the detected line/edge tangent.
- **Max correct** - maximum side correction allowed toward the crosshair.
- **Min conf** - minimum detection confidence required before movement.
- **Direction** - Forward or Reverse along the detected line.
- **Capture after move** - optionally capture the post-move LinuxCNC position into the active manual trace.
- **Follow Step** - performs one bounded follow move.
- **STOP Move** - sends LinuxCNC abort.

## Suggested first test

Use conservative values:

```text
Step: 0.010 or 0.025
Max correct: 0.010 or 0.025
Min conf: 45
Feed/min: 5
```

Workflow:

1. Open LinuxCNC/QtPlasmaC and switch to MANUAL/JOG mode.
2. Home X/Y/Z.
3. Keep torch/plasma disabled.
4. Open Camera Calibrate.
5. Run camera calibration if not already loaded.
6. Place a high-contrast line/edge under the camera.
7. Use Find Line / Edge and verify the overlay and confidence look sane.
8. Check Enable follow.
9. Click Follow Step once.
10. If it moves the wrong way along the line, change Direction.

## Files changed

- `fabscan/app.py`
- `fabscan/camera_calibration.py`
- `fabscan/settings.py`
- `README.md`

## Notes

This is still a proof step. It does not chase the edge continuously. It moves once, stops, and waits for the next button press.
