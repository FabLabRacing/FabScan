# FabScan TODO / Development Roadmap

This file tracks planned work, experiments, and open design questions for FabScan.

It is intentionally separate from:

- `README.md` — what FabScan is and how to get started
- `FabScan Usage Instructions.md` — how to operate it
- `CHANGELOG.md` — what changed between versions
- `HISTORY.md` — why the current architecture exists

FabScan is currently in the **v0.6 alpha** phase. The core belief/connected-path architecture is working in simulation and on the physical LinuxCNC table. The current priority is to validate it against real shop-floor use cases without destabilizing a good baseline.

---

## Next Release — Low-Risk Cleanup / Tooling

### Code cleanup

- [ ] In `path_belief.py`, initialize `heading_innovation = 0.0` near the top of `update()` and remove the fragile `if 'heading_innovation' in locals()` check.
  - Current behavior is valid; this is readability/refactor-safety cleanup rather than a functional bug.

### Generalize SVG simulator input

- [ ] Add **Select SVG → preprocess → choose path/profile → run** workflow.
- [ ] Keep SVG preprocessing isolated from planner/estimator/controller logic.
- [ ] Discover usable paths/profiles rather than assuming fixed A–G names.
- [ ] Handle common SVG primitives and transforms, including:
  - lines / polylines / polygons
  - circles / ellipses
  - arcs / paths
  - splines / Bézier curves
  - groups and transforms
  - open and closed geometry
- [ ] Generate/cache the simulator assets automatically rather than requiring hand-built `*_600ppi.png` and `*_truth.npz` files.
- [ ] Allow the user to select which path/profile in a multi-object SVG should be followed.
- [ ] Preserve the existing A–G test sheet as a known regression dataset.

### Run Review Video Generator

- [ ] Generate an MP4 from captured run frames after a trace completes.
- [ ] Make playback rate configurable; **10 FPS** is a useful diagnostic default.
- [ ] Add center crosshair/reticle.
- [ ] Overlay recorded planner information, where available:
  - step number
  - planner confidence
  - usable look-ahead
  - forward progress
  - lateral correction
  - upcoming turn
  - recommended feed / velocity
  - safety / HOLD state
- [ ] Optionally draw the same belief / connected-path / command overlays used by the live preview.
- [ ] Keep video generation entirely post-run so it adds no tracing-time overhead.

### Documentation

- [ ] Add a **Camera Hardware / Compatibility** section based on the known-good 2017 UVC microscope.
  - Linux driver: `uvcvideo`
  - V4L2 / UVC
  - Known-good FabScan mode: **800×600 MJPG @ 30 FPS**
  - Manual focus
  - Camera also reports MJPG up to 1600×1200 @ 30 FPS
  - It does **not** expose 1280×720
- [ ] Document camera-selection guidance: UVC/V4L2, stable video mode, manual focus, rigid mounting, practical working distance, controllable lighting, MJPG preferred.
- [ ] Mention current replacement cameras only as **likely compatible / unvalidated** until physically tested.

---

## Real-World Validation — Current Priority

Do not tune FabScan only to make the existing G test look perfect. The current alpha should be tested against geometry it was not specifically developed around.

### FabLab real-use-case tests

- [ ] Select several **real parts/templates from the shop** that represent normal FabScan use.
- [ ] Prioritize large outside-edge traces roughly **24×24 in or larger**, since that is a primary intended workflow.
- [ ] Include at least:
  - [ ] one mostly smooth / large-radius outline
  - [ ] one profile with several sharp transitions/notches
  - [ ] one imperfect real shop-floor template, such as hand-cut cardboard
- [ ] Run initial tests with the current alpha settings unchanged.
- [ ] Record each run and judge practical usefulness before chasing small numerical improvements.
- [ ] Compare exported geometry against the physical template / known reference where practical.

A particularly relevant real-world use case is tracing cardboard templates used to fabricate custom aluminum automotive dash panels and similar one-off parts.

### Additional controlled geometry

Once arbitrary SVG input is available, add geometry that broadens the regression set rather than simply making G harder.

- [ ] **H — freeform/spline profile**
  - smooth continuously changing curvature
  - at least one inflection where curvature changes sign
  - gentle and tighter regions
  - no true sharp corners
- [ ] **I — ellipse**
  - continuously changing curvature
  - tests transition from flatter sides into tighter ends
- [ ] Consider a later mixed freeform test with spline → tangent straight → arc transitions.
- [ ] Consider a path-association test with a nearby disconnected profile/edge.

---

## Sharp-Vertex Behavior — Investigate, Do Not Overreact

The latest physical G data shows that most of the trace is already substantially better than the original approximately **0.030 in accuracy goal**. The largest remaining errors are concentrated around a few sharp vertices where the planner begins turning slightly early.

This is currently a **refinement**, not evidence that the v0.6 architecture is failing.

### Low-risk experiments first

- [ ] Test modest step-size reduction.
- [ ] Test modest steering/look-ahead reduction.
- [ ] Test the combination in simulation before physical testing.
- [ ] Compare sharp-corner improvement against radius/freeform performance.
- [ ] Stop tuning if improving sharp corners clearly harms smooth curvature elsewhere.

### Possible future planner work

Only if real-world validation shows the issue materially affects useful traces:

- [ ] Investigate explicit **radius vs sharp-vertex classification** within the connected-path planner.
- [ ] For a true vertex, consider a committed sequence:
  - detect stable incoming/outgoing legs
  - classify/commit to vertex
  - finish incoming leg to vertex
  - latch outgoing leg
  - resume ordinary connected-path following
- [ ] Do **not** resurrect raw-detector-driven Corner Assist behavior.

Guiding principle:

> A detector result is evidence. The model remembers. The planner decides. Motion executes.

---

## Continuous / Velocity-Based Following — Future Roadmap

Continuous following should be approached as a measured transition from the current bounded step-and-settle system, not as simply "run the current loop faster."

### 1. Characterize the stepped-follow ceiling

- [ ] Sweep step length and cycle rate in simulation, then physically.
- [ ] Measure:
  - step size
  - cycle time
  - effective IPM
  - camera/perception time
  - planner time
  - motion time
  - settle time
  - RMS / max error
  - HOLD count/reasons
- [ ] Test at least straight, curved, and mixed geometry.
- [ ] Determine the practical maximum performance of discrete following before changing architecture.

### 2. Measure predictor / dead-reckoning horizon

- [ ] Feed real camera observations at the actual achievable camera rate.
- [ ] Run estimator/predictor updates at a faster rate between observations.
- [ ] Use SVG ground truth to measure error growth versus time since the last camera observation.
- [ ] Measure separately:
  - cross-track error
  - along-track error
  - heading error
  - curvature error
- [ ] Repeat at multiple simulated machine speeds.
- [ ] Determine a practical prediction/latency budget.

Prefer prediction based on the last known **connected path + machine state** rather than simple tangent-only dead reckoning.

### 3. Research LinuxCNC continuous-motion interface

- [ ] Verify the LinuxCNC-native mechanism for external/sensor-driven Cartesian velocity control.
- [ ] Investigate current Cartesian `JOG_CONTINUOUS` / teleop behavior.
- [ ] Verify whether X/Y velocity can be updated repeatedly while already moving without stop/restart behavior.
- [ ] Verify how LinuxCNC applies acceleration limits during changing Cartesian jog velocities.
- [ ] Determine whether another LinuxCNC interface is preferable for externally generated XY trajectory/velocity commands.
- [ ] Confirm any QtPlasmaC-specific state/safety considerations.

FabScan must **not** attempt to become the real-time servo controller. LinuxCNC should retain ownership of the real-time motion layer.

### 4. Build full continuous loop in simulation

- [ ] Extend `machine_dynamics.py` so the planner's velocity recommendation becomes an actual desired velocity/vector.
- [ ] Run a multi-rate simulation:
  - LinuxCNC/machine dynamics at servo period
  - FabScan predictor/controller at an appropriate faster-than-camera rate
  - perception/planner updates at camera rate
- [ ] Treat velocity as a vector (`vx`, `vy`) or equivalent speed + heading state.
- [ ] Test acceleration/deceleration and changing heading under the real machine limits.

### 5. Continuous-mode safety testing in simulation

- [ ] Delayed frames.
- [ ] Dropped frames.
- [ ] Shrinking look-ahead.
- [ ] Confidence collapse while already moving.
- [ ] Sharp turns entered at excessive speed.
- [ ] Controlled deceleration to HOLD.
- [ ] Establish stopping-distance rules that include:
  - observation age / latency travel
  - braking distance
  - safety margin

In continuous mode, **HOLD means controlled deceleration to zero**, not merely declining to issue another step.

### 6. LinuxCNC simulation / physical rollout

- [ ] Exercise the chosen LinuxCNC continuous-motion API in a LinuxCNC simulation configuration first.
- [ ] Log desired vs actual motion behavior.
- [ ] Begin physical testing only at very low velocity.
- [ ] Increase speed based on measured prediction, look-ahead, braking, and real-machine results.

---

## Estimation / Planner Research Ideas

These are possible improvements, not commitments.

- [ ] Compare current estimator against a simple Kalman-style model.
- [ ] Evaluate EKF/Kalman only if it improves measurable behavior on replay and simulator datasets.
- [ ] Improve confidence decomposition and diagnostic reporting.
- [ ] Explore better curvature-state estimation.
- [ ] Explore stronger temporal connected-path association and branch scoring.
- [ ] Improve automatic diagnosis of failures by layer:
  - perception
  - association
  - estimator
  - planner
  - controller
  - LinuxCNC execution

---

## Development Guardrails

These are lessons from the v0.5 → v0.6 development history.

- **Do not overfit the planner to G.** Add new geometry and real use cases before making architecture changes for a small number of G features.
- **Preserve a known-good baseline.** Change one major behavior at a time and regression-test against recorded and simulated datasets.
- **Measure before modeling.** Add physical effects to the realistic simulator only when they are measured or motivated by repeatable real-machine behavior.
- **Keep synthetic stress tests labeled as synthetic.** Do not confuse deliberately injected abuse with measured machine realism.
- **Prefer HOLD over guessing.** Ambiguous path continuation should fail safely and diagnostically.
- **Keep planner, controller, and motion execution separate.** A correct planner command should not be distorted by the executor.
- **Physical testing remains authoritative.** Simulation is a debugging and development bench, not proof that the real machine will behave identically.
- **Remember the intended use case.** FabScan is a practical shop-floor tracing/digitizing tool, not a CMM. Improvements should be judged against useful part/template tracing, not mathematical perfection alone.

---

## Completed / Current Baseline

The following are already part of the current v0.6 alpha baseline and are listed here only to make the roadmap boundary clear:

- Persistent world-space path belief
- Connected-path look-ahead planner
- Follow-run recorder
- Deterministic replay/rerun
- SVG ground-truth simulator
- Measured camera realism model
- Observation-delay simulation
- FabLabPlasma machine-dynamics simulation
- Bounded real-machine belief following
- Separate forward progress and lateral correction
- Nearest qualifying connected-component association
- Coordinated LinuxCNC XY moves
- Independent physical safety gates
- Live planner overlay
- Cleaned simulator UI and direct simulator launcher

---

## Contributing

The items above are intentionally open to discussion. A checkbox does not mean the proposed implementation is settled.

If a contributor has a simpler or more robust approach, especially around path estimation, sharp-vertex classification, LinuxCNC continuous motion, or diagnostics, discussion and experimentation are welcome.

For the reasoning behind the current architecture, see [`HISTORY.md`](HISTORY.md).
