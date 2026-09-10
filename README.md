# FabScan

FabScan is a LinuxCNC companion utility for turning images, camera observations, and CNC-measured points into usable 2D DXF geometry.

The current **v0.6.0-alpha.1** work adds a belief-based camera follower: FabScan observes the profile with a USB camera, maintains a persistent world-space path estimate, looks ahead along the connected profile, plans a bounded X/Y move, applies independent safety checks, and then executes one coordinated LinuxCNC X/Y move. It is intended for digitizing/tracing parts — **not cutting them**.

FabScan remains deliberately limited to X/Y tracing. It does not command Z motion, torch firing, plasma start, spindle output, THC, or cutting cycles.

## Alpha status

`v0.6.0-alpha.1` is an early real-machine alpha. The new planner has completed full simulated traces and long physical traces on the development plasma table, but it is still under active development. In particular, sharp connected-path vertices can still be turned slightly early and are the next major planner target.

Use the real-machine follower cautiously, start with **Dry Plan**, use conservative bounded steps, keep the cutting process disabled, and remain at the machine while testing.

## Main capabilities

- Image contour tracing to DXF
- USB-camera capture
- Manual LinuxCNC point tracing to DXF
- Camera-to-machine X/Y calibration
- Legacy line/edge preview and follow tools
- v0.6 belief-based connected-path following
- Coordinated LinuxCNC X/Y step execution
- Automatic physical-run recording for troubleshooting
- Read-only record/replay tools
- Deterministic rerun support
- SVG ground-truth simulator
- Measured camera-realism model
- Observation-delay model
- Machine acceleration/deceleration model
- Layered simulator DXF export
- Live planner overlay in the real camera preview

## Requirements

FabScan is primarily developed and tested on Linux with LinuxCNC.

Python dependencies are listed in `requirements.txt`:

```text
opencv-python
numpy
Pillow
ezdxf
```

The LinuxCNC Python module must be available when using CNC position, jog, calibration, controlled movement, or physical following.

## Install

Clone or copy the repository, then from the project directory:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 fabscan.py
```

On a LinuxCNC machine, start LinuxCNC first and home the required axes before using machine-motion features.

The development machine launches FabScan with BLAS threading limited and away from the reserved LinuxCNC realtime CPU. That tuning is machine-specific; it is **not a universal FabScan requirement**. See `Usage Notes.txt` for the development-machine example.

## Quick start — image to DXF

1. Start FabScan.
2. Load an image or capture one from a camera.
3. Adjust threshold/cleanup until the desired geometry is isolated.
4. Find contours.
5. Enable only the geometry you want.
6. Set scale using a known dimension.
7. Export DXF.
8. Do final cleanup/CAM work in SheetCam or CAD as needed.

## Quick start — real belief follow (alpha)

The current real-machine follower is in **Camera Calibration**.

1. Start LinuxCNC, home the machine, and make sure the cutting process is disabled.
2. Start FabScan and open Camera Calibration.
3. Open the camera and complete camera calibration.
4. Put the desired profile under the camera crosshair.
5. Set the desired **Start dir**.
6. Verify the normal line/profile detector sees the intended feature.
7. Leave the M6 safety defaults at first:

```text
Hard max move:     0.035 in
Min planner conf:  45 %
Min look-ahead:    0.045 in
```

8. Click **Dry Plan (NO MOVE)** and verify the proposed direction/overlay.
9. Check **ARM real M6 motion** only when ready for physical motion.
10. Run one **Belief Step**.
11. If the physical direction is correct, use **Belief N** with a small Count and increase gradually.

Physical M6 runs force the run recorder ON. The planner's recommended IPM is currently diagnostic only; real motion remains bounded step-and-settle at the configured Follow Feed.

### Real follower architecture

```text
camera frame
    -> existing FabScan perception
    -> persistent world-space path belief
    -> connected-path look-ahead planner
    -> planner safety gate
    -> physical safety gate
    -> progress + tuned lateral correction controller
    -> one coordinated LinuxCNC X/Y G1
    -> settle / observe again
```

The old Corner Assist and Virtual Target logic are bypassed by the v0.6 real belief follower.

## Live planner overlay

Enable **Live planner overlay** in Camera Calibration to draw already-computed planner data on the normal camera preview. The overlay does not rerun perception or planning.

The display shows the current measurement/belief, connected look-ahead, approved command, lateral correction, planner confidence, look-ahead distance, command length, turn information, recommended velocity, and safety/HOLD state.

## SVG simulator

Click **Open SVG Simulator…** from Camera Calibration. The simulator asks for an existing recorded Follow-run folder because it reuses that run's saved detector settings and camera calibration.

The simulator uses exact SVG geometry as ground truth and can run the real FabScan perception, belief, planner, delay model, camera-realism model, and configured machine dynamics without LinuxCNC motion authority.

The simulator UI is organized into:

- **Run** — shape, camera position, Follow Step, Auto Follow, Count, save/DXF
- **Environment** — camera realism, observation delay, machine dynamics
- **Diagnostics** — truth/perception/belief data and post-run summary

Simulated runs can export layered DXF containing SVG truth, camera path, and M4 belief path.

## Record / replay

Physical M6 runs are recorded automatically. Recordings contain frames (when enabled), settings/calibration, event/timeline data, planner/belief state, controller fields, requested targets, actual post-move positions, and coordinated-executor details.

Generated recordings and simulator runs are intentionally not part of source control. Keep useful test datasets separately when needed for regression work.

## Safety

FabScan is experimental machine-control software. Before any real-machine test:

- Disable torch/spindle/cutting outputs.
- Home the required axes.
- Verify LinuxCNC is ready and idle.
- Keep the machine within a clear travel area.
- Start with Dry Plan and one bounded step.
- Remain at the machine with a working E-stop.
- Do not assume a successful simulation proves every real machine will behave identically.

FabScan's v0.6 follower includes bounded-move, confidence/look-ahead, no-reversal, LinuxCNC-state, endpoint, and HOLD safety checks, but these are not a substitute for normal machine safety practices.

## Documentation

See:

- `FabScan Usage Instructions.md` — detailed workflows and current alpha controls
- `CHANGELOG.md` — development/version history
- `Usage Notes.txt` — development-machine launch/realtime notes

## License

MIT. See `LICENSE`.
