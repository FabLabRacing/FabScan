# FabScan v0.5.14 - Follow Settle / Line Quality

v0.5.14 is a camera-follow tuning build. It adds a user-settable settle delay after follow moves and adds live line-quality feedback so camera height, focus, lighting, and search-box size are easier to judge while testing.

## What changed in v0.5.14

- Camera Calibration `Single-Step Follow` now has a saved `Settle ms` field.
  - Default: `150 ms`
  - Allowed range: `0` to `2000 ms`
  - Follow Step / Follow N wait this long after each commanded move before grabbing/evaluating the next line frame.
  - This helps avoid reading the camera while the gantry/camera mount is still ringing after a step move.
- Follow status now includes live line-quality information:
  - detected line span in pixels
  - approximate detected line width in pixels
  - number of points used for the fit
  - current search-box size
- Follow N no longer adds a second fixed wait after each step; the new `Settle ms` setting controls the post-move wait.
- About/title updated to `FabScan v0.5.14 - Follow Settle / Line Quality`.

These changes are aimed at the current printed-line testing where camera distance, camera rigidity, and pixels-per-inch have a large effect on line detection.

## What changed in v0.5.13

- The yellow Camera Calibration crosshair now stays visible all the time.
- The Camera Calibration `Dot marker` checkbox controls only the calibration dot overlay:
  - green detected-dot circle/cross marker
  - red `DOT NOT FOUND` text
- The line/edge overlay remains controlled separately by the existing `Overlay` checkbox.
- Existing v0.5.12 settings migrate cleanly: if the old `camera_calibration_show_crosshair` setting was off, the new dot-marker setting starts off too.

## What changed in v0.5.12

- Camera Calibration Status panel is now fixed-height **and scrollable** so live Line center status wrapping no longer resizes the layout or clips the last status lines.
- Camera Calibration gained a saved overlay checkbox, now refined in v0.5.13 into the `Dot marker` control.

## What changed in v0.5.11

- Camera Calibration Status panel was made taller/fixed-height so live Line center status wrapping does not resize the layout.
- Single-Step Follow now has its own saved Feed setting instead of reusing the calibration feedrate.

## What changed in v0.5.10

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

## Suggested line-follow settings from v0.5.14 testing

For thin printed-line tests with the camera close to the paper:

```text
Mode: Line center
Step: 0.030
Feed: 100 units/min
Settle ms: 150 to 300
Max correct: 0.010 to 0.012
Min conf: 45 to 55
Search px: 175 to 200
Count: 10 to 50
```

For general cautious testing:

```text
Mode: Line center
Step: 0.025 to 0.050
Feed: 5.0 units/min to start, then increase as confidence improves
Settle ms: 150
Max correct: 0.010 or 0.015
Min conf: 55 or 60
Count: 10 to 50
```

If the first follow step goes the wrong way, change Direction and click `Find Line / Edge` again before running Follow N.

## Safety boundary

FabScan remains a camera/image/manual-trace helper for DXF creation. It does not control Z, torch firing, plasma start, spindle, cycle start, THC, or cutting output.
