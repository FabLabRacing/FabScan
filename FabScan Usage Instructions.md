# FabScan Usage Instructions — v0.6.0-alpha.1

FabScan is a LinuxCNC companion utility for creating usable 2D geometry from images, camera observations, or CNC-measured points and exporting DXF for CAD/CAM cleanup.

The current alpha includes both the older tracing tools and the newer **belief-based connected-path follower**. The newer follower is the recommended path for experimental camera-guided physical tracing.

FabScan is intentionally **X/Y tracing only**. It does not command Z, torch firing, plasma start, spindle output, THC, or a cutting cycle.

---

## 1. Starting FabScan

From the FabScan project folder:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 fabscan.py
```

On a LinuxCNC machine, start LinuxCNC first if you plan to use CNC position, jog, calibration, controlled moves, or real following.

The development machine uses a CPU-pinned launch command to keep FabScan away from the LinuxCNC realtime CPU. That is machine-specific; see `Usage Notes.txt`.

---

## 2. Image Contour Tracing

This workflow does not require LinuxCNC motion.

1. Use **Load Image** or **Camera Capture**.
2. Adjust image cleanup controls until the desired part/line is isolated.
3. Use **Show Threshold** to verify what FabScan is actually detecting.
4. Click **Find Contours**.
5. Enable only the desired contours.
6. Use **Set Scale** with a known distance.
7. Use the X/Y sanity check if you know the expected size.
8. Export DXF.
9. Finish cleanup/tooling in SheetCam or CAD.

Keep cleanup values conservative. Excessive blur/noise removal/simplification can change real geometry.

---

## 3. Camera Capture

Open **Camera Capture** from the main FabScan window.

Typical development-camera starting point:

```text
Camera index: 0
Resolution:   800 x 600
```

Use **Open / Restart Camera**, then **Capture Frame** to return a still image to the main tracing window.

If a Linux camera node opens but produces no image, try another camera index or a lower supported resolution. Some `/dev/video*` nodes are metadata rather than actual video streams.

---

## 4. Manual CNC Point Trace

Manual tracing uses LinuxCNC position as the measuring reference.

1. Start LinuxCNC and home the machine.
2. Open/refresh LinuxCNC position in FabScan.
3. Jog to points around the real part/profile.
4. Capture points in order.
5. Choose open/closed trace as appropriate.
6. Edit/delete points if required.
7. Export the manual trace DXF.

This is useful when camera following is unnecessary or when a few manually measured features are easier than vision.

---

## 5. Jog and Controlled X/Y Move

FabScan contains guarded LinuxCNC movement helpers for setup/calibration/tracing.

Before motion, verify:

```text
Machine ON
Interpreter IDLE
Required axes homed
Clear travel area
Cutting output disabled
```

The v0.6 belief follower uses one coordinated LinuxCNC X/Y move per approved physical step rather than separate X and Y jogs.

---

## 6. Camera Calibration

Open **Camera Calibration** before real camera-guided tracing.

General workflow:

1. Open the USB camera.
2. Confirm the requested camera resolution is actually active.
3. Set camera orientation so the transformed preview matches machine X/Y.
4. Detect the calibration dot/target.
5. Run the X/Y calibration movement.
6. Verify **Center Dot** moves the machine in the expected direction.

Do not proceed to automatic physical following until the calibration direction is correct.

---

## 7. Line / Profile Preview

The normal line/profile detector is still used as the first perception stage for the belief follower.

Typical test configuration has used:

```text
Mode:       Line center
Resolution: 800 x 600
```

Put the desired profile reasonably near the camera crosshair and use **Find Line / Edge**/preview tools to make sure FabScan sees the intended feature rather than clutter.

The detector confidence is a quality score, not a dimensional-accuracy percentage.

---

## 8. v0.6 Real Belief Follow (Alpha)

The current physical follower is the **M6.3 Real Belief Follow** section inside Camera Calibration.

It uses:

```text
real camera
  -> line/profile perception
  -> persistent world-space belief
  -> connected-path look-ahead
  -> bounded planner
  -> physical safety gate
  -> forward progress + tuned lateral correction
  -> one coordinated LinuxCNC X/Y G1
  -> settle and observe again
```

The old Corner Assist and Virtual Target logic are bypassed in this path.

### Current M6 safety defaults

Start with:

```text
Hard max move:     0.035 in
Min planner conf:  45 %
Min look-ahead:    0.045 in
```

Keep the existing follow correction tuning conservative. The development tests that produced the first successful long real G trace used approximately:

```text
Follow Step:   0.030 in
Deadband:      0.003 in
Gain:          0.35
Max Correct:   0.003 in
```

Do not treat these as universal machine settings; camera scale, mechanics, feed, and profile geometry matter.

### First physical test

1. Home LinuxCNC and disable all cutting outputs.
2. Complete camera calibration.
3. Put a clean simple profile under the camera.
4. Set **Start dir** to the desired direction of travel.
5. Check **Enable follow**.
6. Leave **ARM real M6 motion** OFF.
7. Click **M6.3 Dry Plan (NO MOVE)**.
8. Inspect the live planner overlay/status and confirm the proposed direction is sensible.
9. Check **ARM real M6 motion**.
10. Run **M6.3 Belief Step** once.
11. Verify the physical move direction.
12. Use **M6.3 Belief N** with a small Count.
13. Increase Count only after behavior is predictable.

Each Belief N run keeps the belief/planner state across steps. Separate Belief Step clicks intentionally start a fresh session.

### What can cause a HOLD

The alpha follower intentionally refuses to guess. HOLD reasons include:

- no usable line/profile detection after retry
- detector confidence below the current detector minimum
- no qualifying connected dark component near the expected profile
- planner rejection / no safe continuation
- planner confidence below the M6 minimum
- usable look-ahead below the M6 minimum
- connected-path progress reversing established travel
- approved move collapsing below a useful size
- LinuxCNC state/motion failure

A HOLD is preferable to silently choosing an uncertain branch.

### Coordinated X/Y execution

Since M6.2, diagonal physical planner vectors are executed as one coordinated LinuxCNC X/Y move. FabScan waits for the requested endpoint and interpreter IDLE, then explicitly restores MANUAL mode before taking the next camera observation.

The planner's **recommended IPM** is currently logged/displayed only. The real follower remains bounded step-and-settle and uses the configured Follow Feed for physical motion.

---

## 9. Live Planner Overlay

Enable **Live planner overlay** in Camera Calibration.

The overlay draws only data already computed by the current M6 decision cycle; it does not rerun perception/planning.

The display includes:

- stabilized/current measurement
- M4 belief estimate
- connected-path look-ahead
- forward command
- lateral correction
- approved endpoint
- planner confidence
- usable look-ahead
- move length
- upcoming turn
- recommended velocity
- candidate count
- safety/HOLD status

If a run HOLDs, leave the preview visible when possible. The final geometry and reason can be useful when comparing the live event to the recorder logs.

---

## 10. Physical Run Recorder

Physical M6 runs force recording ON.

A recording can contain:

```text
manifest.json
settings.json
calibration.json
events.jsonl
steps.jsonl
timeline.csv
writer_status.json
frames/*.png            (when frame capture is enabled)
```

M6 data includes belief state, tangent/curvature, usable look-ahead, planner confidence, controller forward/correction components, requested target, coordinated executor command, endpoint/IDLE result, and actual post-move position.

Generated run folders belong in the project's replay/output area and are intentionally ignored by source control.

When sharing a run for numeric analysis, frames are often unnecessary. Keep them when investigating perception/association/image glitches.

---

## 11. Replay and Deterministic Rerun

FabScan's development tools support read-only inspection of previously recorded runs and deterministic rerun of detector/planner logic against saved decision frames.

Replay is valuable for:

- regression testing
- comparing planner changes against the same real camera evidence
- locating a specific HOLD or confidence collapse
- inspecting belief/look-ahead evolution

A replay cannot perfectly validate an alternate full physical trajectory after the new planner diverges from the path that originally produced the recorded camera frames. The SVG simulator exists to solve that limitation.

---

## 12. SVG Ground-Truth Simulator

From Camera Calibration click **Open SVG Simulator…**.

Select an existing recorded Follow-run folder. The simulator uses that run only as a source for detector settings and camera calibration.

The simulator uses the bundled `simulator_assets/TestImage.svg` as exact world geometry and can move a virtual camera wherever the planner chooses.

### Run tab

Contains the main simulation controls:

- shape selection
- camera position/centering
- Follow Step
- Run Auto Follow
- Pause
- Count
- Save Run / DXF

### Environment tab

Contains realism controls:

- camera realism
- observation delay
- machine dynamics

The measured development baseline includes a camera appearance model derived from real A–G recordings, about 120 ms of observation delay, and machine dynamics based on the development LinuxCNC configuration.

### Diagnostics tab

Contains ground-truth/current-pose information, perception/belief data, and the post-run summary.

### Simulated run export

Saved simulator runs include layered DXF and diagnostic data. Typical DXF layers are:

```text
SVG_TRUTH
CAMERA_PATH
M4_BELIEF
```

Other files can include planner steps, camera path, belief path, SVG truth reference, settings/calibration, camera realism, timing model, and machine dynamics.

The simulator is a development/troubleshooting bench, not proof that an arbitrary physical machine will behave identically.

---

## 13. Current Known Alpha Limitation

The first long physical M6.2 G trace showed that the architecture follows the real profile far better than the older follower, but a few sharp corners are still turned slightly early.

The remaining error is concentrated at sharp connected-path transitions rather than general wandering. This is the next major planner-development target.

Do not compensate for that limitation by blindly lowering safety/confidence thresholds.

---

## 14. Settings and Generated Data

FabScan user settings are normally stored outside the repository, under the user's platform-specific config directory (for Linux, typically `~/.config/fabscan/settings.json`).

The repository `.gitignore` intentionally excludes generated data such as:

```text
venv/
exports/
replays/
simulated_runs/
*.log
*.bak*
```

Do not commit captured runs, installer backups, or generated DXF simply because they are present in the project directory.

The bundled files in `simulator_assets/` are different: those are source/test assets required by the current simulator and should remain version-controlled.

---

## 15. Troubleshooting

### M6 Dry Plan looks good but physical run immediately loses confidence

Make sure the current alpha contains the M6.0.1 start-direction seed fix. The first estimator tangent must be oriented from the selected Start dir so the second observation is not falsely interpreted as an approximately 180-degree contradiction.

### HOLD: no connected dark component near profile

A current alpha should contain the M6.2 valid-component association fix. Tiny undersized scratches/specks are filtered before selecting the nearest qualifying profile component.

If the HOLD remains, inspect the recorded frame and timeline; a genuine loss of the profile can still produce the same general class of message.

### Physical diagonal move feels like X then Y

A current alpha should contain coordinated XY execution (M6.2+). Each approved diagonal step should be one LinuxCNC X/Y G1, not two sequential axis jogs.

### Camera opens but no image appears

Try another camera index or a supported lower resolution such as 800x600 or 640x480.

### LinuxCNC motion is refused

Check that LinuxCNC is connected, the machine is ON, required axes are homed, the interpreter is IDLE, and the requested motion mode is permitted.

### Wrong initial direction

Do not simply arm the machine and retry. Set the intended Start dir and use **Dry Plan (NO MOVE)** first.

---

## 16. Practical Alpha Test Order

For a new machine/setup:

1. Image tracing only
2. LinuxCNC position readout
3. Manual jog/controlled move
4. Camera calibration
5. Line/profile preview
6. Dry Plan only
7. One Belief Step on a straight line
8. 5–10 Belief N steps
9. Longer straight/radius test
10. Complex geometry only after the basic cases are predictable

Keep cutting outputs disabled throughout development tracing tests.

---

## 17. Development Baseline

The development plasma-table PC has been tested with FabScan deliberately kept away from the isolated LinuxCNC realtime CPU. The local launch example is documented in `Usage Notes.txt`.

This scheduling arrangement is **not required by FabScan itself** and should not be copied blindly to a different LinuxCNC computer. Choose CPU affinity based on that machine's realtime configuration.
