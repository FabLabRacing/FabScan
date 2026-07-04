# FabScan v0.5.10 - Camera Robustness

v0.5.10 builds on the v0.5.9 follow-direction latch and focuses on making the USB camera path more forgiving on the LinuxCNC/FabScan machine.

## What changed

- Default camera size changed from `1280 x 720` to `800 x 600`.
- Saved legacy `1280 x 720` camera settings are migrated to `800 x 600` on load.
- Camera Capture and Camera Calibration now have resolution presets:
  - `800 x 600`
  - `640 x 480`
  - `1280 x 960`
  - `1600 x 1200`
  - `1280 x 720`
- Camera status now reports requested size, actual size, backend, and format.
- Linux camera open explicitly requests the V4L2 backend.
- OpenCV asks for MJPG and a one-frame buffer when the backend supports it.
- Preview frame reads now happen in a background thread so slow/failing camera reads are less likely to freeze Tkinter.
- If no frames arrive, FabScan reports likely causes such as a metadata `/dev/video` node, unsupported resolution, busy camera, or disconnected camera.
- Preview `PhotoImage` objects are tied to the dialog window, and preview callbacks are cancelled on close.
- Removed confirmation popups for:
  - enabling jog controls
  - enabling controlled moves
  - running camera calibration

Motion safety checks remain in place: FabScan still refuses guarded motion unless LinuxCNC status is acceptable for that motion path. FabScan remains X/Y only: no Z, no torch, no plasma, and no cycle start.

## Suggested camera settings

For the current microscope camera, start with:

```text
Camera index: 0
Resolution: 800 x 600
```

Avoid `1280 x 720` unless the status line confirms the camera actually returns that size reliably.

## Suggested line-follow settings from v0.5.9

```text
Mode: Line center
Step: 0.050
Max correct: 0.010 or 0.015
Min conf: 55 or 60
Count: 10 to 50
```

If the first follow step goes the wrong way, change Direction and click `Find Line / Edge` again before running Follow N.
