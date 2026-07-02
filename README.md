# FabScan v0.5.9 - Follow Direction Latch

v0.5.9 fixes the back-and-forth motion seen during bounded multi-step line following.

A detected line has no inherent arrow direction. Depending on the camera frame and fitted contour, the detected tangent can flip 180 degrees from one step to the next. In v0.5.8 that could make `Follow N` alternate forward/backward even though the line detection and calibration were otherwise working.

## What changed

- Added a machine-space follow heading latch.
- First follow step uses the Forward / Reverse selector.
- Later follow steps choose the detected tangent direction closest to the previously successful move direction.
- `Find Line / Edge`, camera/threshold/search setting changes, Direction changes, and STOP clear the latch.
- Follow status now reports whether the heading is `new heading` or `latched`.
- Motion remains guarded X/Y incremental jog only.

## Suggested test

Use the same settings that exposed the direction flip:

```text
Mode: Line center
Step: 0.050
Max correct: 0.010 or 0.015
Min conf: 55 or 60
Count: 10 to 50
```

Workflow:

```text
1. Open Camera Calibrate.
2. Verify or run calibration.
3. Put the line under the camera.
4. Click Find Line / Edge.
5. Set Direction to Forward or Reverse.
6. Click Follow Step once and verify the direction.
7. Click Follow N.
```

If the first step goes the wrong way, change Direction and click `Find Line / Edge` again before running Follow N.

