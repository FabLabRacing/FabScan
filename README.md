## v0.5.3.3 - Fresh Frame Retry / Sampled Overlay Warning

- Added a small transient not-found recovery path for Follow Step / Follow N. If line/edge detection returns a complete `not_found`, FabScan waits for a fresh camera frame and retries before refusing the step.
- The retry is intentionally narrow: it only retries no-line/no-edge frames. It does not bypass confidence, sanity, progress lock, correction limits, or LinuxCNC readiness checks.
- Post-move detection uses the same fresh-frame retry so a single bad/undrawn frame does not immediately poison the next follow step.
- The timeline CSV now records detection phase, retry attempt, retry wait time, and whether a fresh frame was received. Post-move detection now has a final summary row after retries.
- Live preview overlay now warns when preview FPS is much lower than capture FPS, because the displayed overlay is only a sampled view and Follow may be using newer frames.
- No virtual-target math, delayed-position math, DXF export, corner assist, Z/torch/plasma behavior, or LinuxCNC command style was intentionally changed.

## v0.5.3.2 - Stage 2D Virtual Target Lite

- Added an experimental `Virtual target` checkbox and `Min prog %` setting in Camera Calibration > Single-Step Follow.
- This is a cautious Stage 2D-lite step, not a velocity-jog or continuous controller. FabScan still sends bounded X/Y position moves only.
- When enabled, FabScan shapes the delayed/current camera-derived target through a small virtual target and decomposes the command into tangent and side components. The goal is to let delay compensation help steering without allowing it to cancel most of the forward step.
- `Min prog %` defaults to 70%, so a normal follow step should keep at least 70% of the requested forward progress unless an existing safety/refusal stops the move.
- Timeline logs now include virtual-target fields such as virtual state, desired target, shaped target, raw/final tangent component, raw/final side component, progress floor, and side limit.
- Defaults remain conservative: delayed position is still off unless enabled, and virtual target is off unless enabled.
- No velocity-jog loop, virtual-machine free-run, Z/torch/plasma control, corner-assist behavior change, or DXF export change was intentionally added.
- About/title updated to `FabScan v0.5.3.2 - Stage 2D Virtual Target Lite`.

## v0.5.3.1 - Position Delay Experiment

- Adds an experimental Stage 2B/2C position-history path in Camera Calibration. FabScan keeps a short rolling LinuxCNC position history during follow moves, jog waits, and post-move checks.
- Adds `Use delayed pos` and `Delay ms` controls to Single-Step Follow. The old behavior remains the default with delayed position OFF.
- When delayed position is enabled, FabScan matches the current camera frame timestamp to a delayed/interpolated LinuxCNC position sample, plans the camera-derived move from that historical position, then converts it back into a safe incremental command from the current position. The command is still bounded by the existing Follow Step + Max correct limit and progress lock.
- Timeline CSV rows now include run/session IDs, run step numbers, delayed-position fields, target-base position, original frame move, and command adjustment values.
- Follow N now starts a fresh run ID and resets the follow/progress latch at the start of each new Follow N, avoiding stale-latch behavior between separate tests.
- No velocity-jog loop, virtual machine target, Z motion, torch/plasma control, corner-assist behavior, or DXF export changes were intentionally added.
- About/title updated to `FabScan v0.5.3.1 - Position Delay Experiment`.

## v0.5.3.0 - Stage 2 Timeline Logging

- Starts Stage 2 as instrumentation only. No follow math, jog behavior, corner logic, or DXF export behavior is intentionally changed.
- Camera Calibration now has a `Timeline log` checkbox in the Single-Step Follow panel. When enabled, FabScan writes a CSV under `~/FabScan Logs/`.
- The timeline CSV records Follow Step / Follow N events including frame sequence, frame age, LinuxCNC status-read timing, pre-move position, detection confidence/offset/angle, move plan, jog timing, settle timing, post-move position, and post-move detection.
- This is meant to help measure the camera/machine-position delay problem before attempting virtual-target or velocity-follow control.
- About/title updated to `FabScan v0.5.3.0 - Stage 2 Timeline Logging`.

## v0.5.26 - Safe Preview / Frame Dropping

- About/title updated to `FabScan v0.5.26 - Safe Preview / Frame Dropping`.
- Added separate camera `Capture FPS` and `Preview FPS` controls. Capture can keep reading at a low rate while the Tk/PIL/ImageTk display work runs even slower.
- Added a `LinuxCNC Safe` preview preset. In Camera Calibration it sets capture to 5 fps, preview display to 1 fps, disables line preview, dot marker preview, mask preview, Corner pause, and Corner assist. In Camera Capture it sets capture to 5 fps and preview display to 2 fps.
- Camera preview loops now use queue-size-one/latest-frame behavior with sequence numbers. They drop stale/repeated frames and do not resize/convert/draw/ImageTk-update unless a new preview frame is actually due.
- Camera wait/pump loops can still keep `current_frame_bgr` fresh for follow/calibration, but they no longer have to redraw the GUI every 50 ms.
- When LinuxCNC Safe preview is enabled, FabScan attempts `cv2.setNumThreads(1)` to keep OpenCV/native processing from spreading across cores.
- Added a `Profile` checkbox for preview timing logs. Camera Calibration profiling reports transform, dot detection, line detection, mask/RGB conversion, resize, overlay, ImageTk, total frame time, achieved capture FPS, preview count, and replaced/dropped frame count.
- No intentional changes to follow motion math, progress lock, corner assist, or DXF export. This release is about reducing and instrumenting preview/display CPU load after LinuxCNC isolation testing showed FabScan preview was the trigger, not raw UVC streaming.

## v0.5.25 - RT-Friendly Camera / Saved Knobs

- About/title updated to `FabScan v0.5.25 - RT-Friendly Camera / Saved Knobs`.
- Camera preview reader now caps background reads at a saved `FPS cap` value, default 20 fps, instead of reading as fast as OpenCV/camera buffering allows.
- Camera reader uses `stop_event.wait()` for capped sleeps and failed-read backoff so camera shutdown stays responsive.
- Camera Capture and Camera Calibration expose/save the shared camera `FPS cap` setting.
- Corner/Hough/intersection detection is skipped unless `Corner pause` is enabled, so straight-line/curve testing with corner handling off avoids unnecessary per-frame CPU work.
- Camera calibration/follow settings are saved immediately when the calibration dialog closes, including the growing follow-control knob set, so the next FabScan session starts with the previous test settings.
- No intended change to line-follow motion math, progress lock, corner assist, or DXF export.

## v0.5.24 - Progress Lock / Latch Fix

- Fixed a follow-latch reset bug: reading the Start dir control no longer writes the same value back to the Tk variable every step, which could fire the variable trace and clear the heading/progress latch during Follow N. This likely prevented the v0.5.23 reverse guard from seeing the prior move.
- Progress continuity now uses the last successful commanded move before the older heading latch when both are available.
- Replaced the silent tangent-only reverse fallback with a hard, visible Progress Lock: if the final commanded move would reverse relative to recent travel, the move is refused before LinuxCNC motion is sent. Status reports the progress dot value, e.g. `progress +0.93` or `dot -0.98` when refused.
- No corner/intersection-assist behavior was intentionally expanded in this release. This is focused on stopping the E/F back-and-forth sticking behavior from reaching the DXF.
- About/title updated to `FabScan v0.5.24 - Progress Lock / Latch Fix`.

## v0.5.23 - Tangent Direction Continuity

This build targets the back-and-forth "stuck" behavior seen on the standard test sheet, especially the radius/tangent transition and circle tests.

- Added stronger signed tangent continuity for follow stabilization.
  - `cv2.fitLine` gives a line axis, not a travel arrow. FabScan now orients the fitted pixel tangent against the current latched machine-space travel heading before applying the EMA angle filter.
  - This reduces the chance that a valid-looking line fit silently becomes a 180-degree reversed motion command.
- Added a commanded-move reverse guard.
  - After tangent and side correction are combined, FabScan checks whether the resulting move would reverse against the previous successful travel direction.
  - If the combined move tries to reverse, FabScan strips side correction once and sends the tangent-only move when that is safe.
  - If the tangent itself would still reverse, the step is refused and the user is asked to use `Find Line / Edge` if the reversal is intentional.
- The last successful move direction is now remembered separately from the line-fit latch so full curves/circles have an extra continuity reference.
- Corner Assist remains unchanged; this release is focused on normal straight/curve follow stability.
- About/title updated to `FabScan v0.5.23 - Tangent Direction Continuity`.

## v0.5.22 - Follow Stabilization

This build pauses corner expansion work and strengthens the normal follow detector underneath it.

- Added `Stabilize` controls under Single-Step Follow.
  - Default: enabled.
  - `Err α` default: `0.35` for EMA filtering of raw pixel offset.
  - `Ang α` default: `0.25` for EMA filtering of detected tangent direction.
  - `Sanity°` default: `30`; after a heading is latched, a non-corner frame whose heading jumps beyond this is rejected instead of becoming motion. Set to `0` to disable this sanity reject.
- Added contour continuity scoring.
  - Once a line/edge has been accepted, contour selection now penalizes candidates that jump far from the previous filtered offset or disagree with the latched/filtered heading.
  - This is meant to reduce silent jumps to the other edge of a line or another nearby feature.
- Follow motion now uses the filtered offset/angle before applying Deadband/Gain/Max correct.
- `Find Line / Edge`, Start dir changes, preview/search setting changes, and STOP reset the follow filter/latch cleanly.
- Corner Assist from v0.5.21 remains present, but this version is deliberately staged before Lucas-Kanade optical flow, Kalman filtering, or PID/P+D control.
- About/title updated to `FabScan v0.5.22 - Follow Stabilization`.

## v0.5.21 - Corner Intersection Assist

This build adds the first practical corner-following assist for Camera Calibration / Single-Step Follow.

- Added `Corner assist` under Single-Step Follow.
  - When `Corner pause` and `Corner assist` are enabled, FabScan tries to use the two Hough-detected line directions to compute the actual corner/intersection point.
  - If the computed intersection is safe and close enough, FabScan drives the camera/crosshair to that intersection instead of stopping and forcing manual Find/Step driving through the corner.
  - After the corner move succeeds, FabScan latches the selected outgoing `Start dir` (`X+`, `X-`, `Y+`, `Y-`) so the next follow step continues down the new leg.
- Added `Lookahead` steps for Corner Assist.
  - Default: `5`.
  - The maximum corner-intersection move is `Lookahead × Step`, clamped to 1–10 lookahead steps.
  - If the calculated corner is farther away than the lookahead window, FabScan refuses the assist and falls back to the safe pause behavior.
- The live overlay/status now reports and marks the fitted corner intersection when available.
- `Follow N` count is no longer capped at 50.
  - It now accepts up to `9999` steps for longer full-shape tests.
  - Each individual move is still bounded by the normal step/correction/corner-assist safety checks, and `STOP Move` remains available.
- About/title updated to `FabScan v0.5.21 - Corner Intersection Assist`.

## v0.5.20 - Live Entry Fix

Small Camera Calibration usability fix:

- Search px now tolerates partial typing/backspacing while Overlay is enabled.
- This prevents Tkinter callback errors when the Search px box is temporarily empty during live preview.
- No motion, corner, or detection behavior was intentionally changed from v0.5.19.

# FabScan v0.5.19 - Corner Resume / Entry Fix

v0.5.19 is a small fix build for Camera Calibration corner-follow testing. It fixes the Corner° field so it can be edited normally while Overlay is enabled, and it makes the corner-pause continuation workflow usable after clicking Find Line/Edge at a corner.

## What changed in v0.5.19

- Fixed the `Corner°` entry typing bug.
  - It no longer clamps/replaces the value while the user is actively typing.
  - This fixes the behavior where backspace could force the field to `10`, then later typing could force it to `135`, especially while the live line overlay was enabled.
- Improved corner-pause resume behavior.
  - When corner pause stops at a possible corner/intersection, choose the next `Start dir`, then click `Find Line / Edge`.
  - If the frame still contains a corner candidate, FabScan arms a one-shot resume: the next follow move bypasses corner pause once so it can step out of the ambiguous L-shaped search area.
  - During that one resume step, FabScan can choose between the Hough primary/secondary corner directions instead of only the averaged line fit.
  - Side correction is skipped for that single corner-resume step, because the normal line-center correction may be based on an averaged/diagonal corner fit.
- No changes to jog, calibration motion, or DXF export.
- About/title updated to `FabScan v0.5.19 - Corner Resume / Entry Fix`.

## What changed in v0.5.18



- Camera Calibration right-side control panel now has its own vertical scrollbar.
- Dot Center Jog and Single-Step Follow controls remain in the same right column, but the column can scroll on shorter displays.
- Mouse wheel scrolling works when the pointer is over the right-side control panel.
- No follow/detection/motion logic changed from v0.5.17.
- About/title updated to `FabScan v0.5.18 - Calibration Right Panel Scroll`.

## What changed in v0.5.17

- Camera Calibration `Single-Step Follow` now has a saved `Corner pause` option.
  - Default: enabled.
  - When FabScan sees two strong line directions in the search box, it pauses/refuses the next follow move instead of blindly fitting one diagonal/averaged line through the corner.
- Added saved `Corner°` setting.
  - Default: `55°`.
  - This is the minimum angle between detected line directions before the corner/intersection pause can trigger.
- The line/edge status now reports secondary-direction/corner information when available, for example `corner 89°/0.42`.
- The old `Forward` / `Reverse` follow direction selector is replaced with `Start dir`: `X+`, `X-`, `Y+`, or `Y-`.
  - First follow step chooses the fitted-line direction whose machine-space projection matches the selected axis/sign.
  - Later steps still latch to the previous successful machine-space heading.
  - This makes testing more repeatable; for example, a mostly Y move can always start as Y+ or Y- instead of depending on an arbitrary fit-line sign.
- About/title updated to `FabScan v0.5.17 - Corner Pause / Axis Latch`.

This is intended to address the sharp-corner tests where a square search box sees both legs of an L-shaped corner and rounds/blends through it. The first implementation is deliberately conservative: pause at the corner, then let the user reposition/select the next start direction and continue.

v0.5.16 is a camera-follow tuning/UI build. It adds side-correction deadband/gain so Line center follow stops chasing tiny frame-to-frame offsets, and it makes the Camera Calibration status box readable while live updates are still coming in.

## What changed in v0.5.16

- Camera Calibration `Single-Step Follow` now has saved `Deadband` and `Gain` fields.
  - `Deadband` default: `0.003` machine units.
  - `Gain` default: `0.50`.
  - `Max correct` remains the hard side-correction limit.
- Side correction now behaves like this:
  - if the raw correction is inside the deadband, no side correction is applied;
  - if it is outside the deadband, the deadband is subtracted and the remaining correction is multiplied by Gain;
  - the result is still limited by `Max correct`.
- Follow status now reports raw correction vs applied correction, plus the active deadband/gain values.
- The Camera Calibration status panel no longer snaps back to the bottom while the user has scrolled up to read older/top lines. It still auto-scrolls when already at the bottom.
- About/title updated to `FabScan v0.5.16 - Follow Deadband / Status Scroll`.

This is intended to reduce the edge-to-edge hunting seen in Line center mode, especially on thin printed lines where small detection jitter can otherwise cause side correction on every step.

## What changed in v0.5.15

- Camera Calibration `Single-Step Follow` now has a saved `Max turn°` field.
  - Default: `70°`
  - Allowed range: `0°` to `180°`
  - `0°` disables the heading-change stop.
- After the first follow heading is latched, each next detected heading is compared to the previous successful machine-space heading.
- If the detected heading change is greater than `Max turn°`, FabScan refuses the move and stops Follow N.
- The stop message tells the user to click `Find Line / Edge` to reset the latch when the stop is an intentional corner.
- About/title updated to `FabScan v0.5.15 - Heading Change Stop`.

This is intended to catch the sharp-corner case where a large search box sees both legs of a corner and starts averaging/rounding through it.

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

## Suggested line-follow settings from v0.5.17/v0.5.18 testing

For thin printed-line tests with the camera close to the paper:

```text
Mode: Line center
Step: 0.030
Feed: 100 units/min
Settle ms: 150 to 300
Max turn°: 50 to 75
Corner pause: enabled
Corner°: 55
Start dir: X+/X-/Y+/Y- based on intended first machine direction
Max correct: 0.005 to 0.012
Deadband: 0.002 to 0.005
Gain: 0.35 to 0.75
Min conf: 45 to 55
Search px: 175 to 250
Count: 10 to 50
```

For general cautious testing:

```text
Mode: Line center
Step: 0.025 to 0.050
Feed: 5.0 units/min to start, then increase as confidence improves
Settle ms: 150
Max turn°: 70
Corner pause: enabled
Corner°: 55
Start dir: choose X+/X-/Y+/Y- for the intended first move
Max correct: 0.010 or 0.015
Deadband: 0.003
Gain: 0.50
Min conf: 55 or 60
Count: 10 to 50
```

If the first follow step goes the wrong way, change `Start dir` and click `Find Line / Edge` again before running Follow N.

## Safety boundary

FabScan remains a camera/image/manual-trace helper for DXF creation. It does not control Z, torch firing, plasma start, spindle, cycle start, THC, or cutting output.
