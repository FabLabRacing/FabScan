# FabScan v0.5.2 - Calibration Screen Jog Controls

This release adds X/Y jog buttons directly to **Camera Calibration Lite** so you can center the calibration dot without leaving the calibration window.

## Why

v0.5.1 proved the calibration loop:

```text
find dot
jog X+
find dot again
jog back
jog Y+
find dot again
jog back
calculate camera/machine transform
```

But the workflow was clunky because centering the dot still required using the main FabScan jog panel or QtPlasmaC jog controls. Since the calibration screen already has the live camera view and dot finder, the small jog controls belong there.

## New behavior

The Camera Calibration Lite screen now includes a **Dot Center Jog** panel:

```text
Step
.001 .005 .010 .050 .100
Y+
X-  Find  X+
Y-
```

The jog buttons use the same guarded LinuxCNC MANUAL-mode incremental jog path as the main FabScan jog controls.

## Safety / limits

```text
X/Y only
Incremental jogs only
No Z
No torch
No program start
Requires LinuxCNC ON / IDLE / MANUAL
Requires X/Y/Z homed
Feed limited by the same FabScan jog guardrails
```

## Suggested workflow

```text
Put the dot under the camera
Open Camera Calibrate
Use Mask view / Threshold until FabScan sees the dot
Use Dot Center Jog to put the crosshair near the dot
Click Find Dot
Click Run Calibration
```

The jog step is saved separately from the calibration move distance.

## Files changed

```text
fabscan/app.py
fabscan/settings.py
fabscan/camera_calibration.py
README.md
```
