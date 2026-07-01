# FabScan v0.5.3 - Center Dot Calibration Test

This release adds **Center Dot** to the Camera Calibration Lite screen.

v0.5.0 through v0.5.2 proved that FabScan can:

```text
find the calibration dot
jog X/Y in MANUAL mode
see how the dot moves in the camera
calculate the camera/machine transform
center the dot from the same calibration screen
```

v0.5.3 uses that transform in the opposite direction:

```text
dot pixel error from crosshair -> guarded X/Y machine correction
```

LinuxCNC is still the ruler and motion controller. The camera is only being used as the steering eye.

## New behavior

The Camera Calibration Lite screen now includes:

```text
Max center
Center Dot
```

After a valid calibration exists, click **Center Dot** and FabScan will:

```text
find the calibration dot
measure its pixel offset from the crosshair
convert that pixel offset into an X/Y machine correction
limit the move to Max center
send guarded MANUAL-mode X/Y incremental jogs
look at the dot again and report the new offset
```

This is a single correction move. If the dot is still a little off, click **Center Dot** again to sneak up on it.

## Suggested test

```text
1. Put QtPlasmaC/LinuxCNC in MANUAL/JOG mode.
2. Home X/Y/Z.
3. Disable torch/plasma.
4. Open Camera Calibrate.
5. Center the dot reasonably using the on-screen jog buttons.
6. Run Calibration.
7. Jog the dot visibly off-center.
8. Set Max center to something conservative, such as 0.050 or 0.100.
9. Click Center Dot.
10. Verify the dot moves toward the crosshair.
```

## Safety / limits

```text
X/Y only
Incremental jogs only
MANUAL mode required
LinuxCNC must be ON / IDLE
X/Y/Z must be homed
No Z
No torch
No program start
Center correction is limited by Max center
```

## Files changed

```text
fabscan/app.py
fabscan/settings.py
fabscan/camera_calibration.py
README.md
README_v0.5.3.md
```
