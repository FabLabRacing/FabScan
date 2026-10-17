# M6.4 sharp-vertex commitment

Implementation identifier: `0.6.0-alpha.1-m6.4.0`. The camera controls and overlay show **M6.4**.

This addresses the Edge-mode failure where a visible ~90° turn shortens each step but the camera continues along the incoming leg until the boundary seed gate stops the run. It applies only to connected **boundary** paths; ordinary Line-mode centerlines retain their existing behavior.

## Behavior

1. **FOLLOW:** use ordinary connected-boundary planning and tuned lateral correction.
2. **CANDIDATE:** two straight connected legs suggest a localized vertex. Require a matching fit in a second distinct frame before commitment.
3. **APPROACH:** freeze the vertex and both leg directions in machine coordinates. Each bounded move ends on the approach to that target; no move blends through the vertex.
4. **OUTGOING:** after an executed pose reaches the vertex, discard the incoming trailing-heading history and latch the outgoing leg. Follow it to a fixed exit target.
5. **FOLLOW:** after an executed pose reaches the exit target, resume ordinary planning with outgoing travel established.

Repeated planning, including Dry Plan, cannot declare arrival. A failed executor does not advance the state. New physical runs and planner reset clear commitment. A completed vertex is suppressed locally so it cannot immediately retrigger.

The ordinary controller still uses its existing gain, deadband and correction limit. In APPROACH/OUTGOING, the controller uses the planner's bounded fixed endpoint directly, without applying those blend/correction controls again. The physical hard move cap and reversal check still apply. A smaller physical cap is supported: state advancement uses actual reported poses, never the originally planned distance.

## Credibility and bounds

These are internal geometry constants, not new UI settings:

- Sharp heading change: **55–135°**.
- Each fit examines **0.080 inch** on either side, excluding the central neighborhood; about 0.06 inch of straight support per leg is needed.
- Straight-leg residual tolerance: **1.5 pixels**, bounded to **0.0015–0.004 inch**. Each leg's two half-fits must agree within **12°**.
- The leg intersection must lie on the original boundary within 1.5 times that residual tolerance. Normal path resampling is deliberately excluded from this test, because it can shave off a real corner.
- Two distinct observations must agree within **0.006–0.010 inch**, calibration dependent, and **12°** on both directions.
- Commit only with the vertex forward and within **0.220 inch**, and camera distance from the incoming line no greater than Max Correct + **0.015 inch**.
- Executed-pose arrival tolerance: **0.003 inch**, matching the existing minimum useful move scale. Planned final steps aim at the vertex itself; the tolerance does not authorize crossing it.
- Outgoing exit target: **0.060 inch** beyond the vertex.
- Every committed step rechecks both 0.080-inch legs against the current gated boundary, with a maximum **4-pixel** residual at sampled support points. Missing or changed support produces HOLD.

The seed-to-boundary gate is unchanged: 10% of Search px, bounded to 15–80 pixels (**40 pixels at Search 400**). There is no centerline fallback for a region-like Edge-mode component. Detector confidence, planner confidence, physical move cap, and the LinuxCNC endpoint executor remain active. Feed remains the configured Follow Feed; recommended velocity remains diagnostic only.

Corners with insufficient visible straight support, very short adjoining legs, heavily rounded/chipped vertices, or turns outside the supported range may remain ordinary-follow cases or HOLD. This is a bounded first implementation, not proof of reliable tracing for every mixed shape.

## Reading a test timeline

`M6_VERTEX_STATE` records each planner decision, including candidates, ordinary-follow classification reasons and refusals. `M6_VERTEX_TRANSITION` records discrete events:

| Event | Meaning |
| --- | --- |
| `CANDIDATE` | First credible vertex observation |
| `CANDIDATE_REJECTED` | Observation changed or support was insufficient before commitment |
| `COMMITTED` | Second distinct matching observation froze the target and outgoing leg |
| `VERTEX_REACHED` | Reported executed pose reached the target |
| `OUTGOING_LATCHED` | Incoming heading retired; outgoing leg now controls travel |
| `RESUMED` | Executed pose established the outgoing leg |
| `HOLD` | Commitment could not safely proceed; reason and target retained |

CSV columns include `planner_version`, `motion_mode`, `vertex_state`, `vertex_id`, `vertex_event`, `vertex_previous_state`, `vertex_reason`, target X/Y, distance, incoming/outgoing vectors, turn, fit error, confirmation count, observation identity and exit target/distance. `MOVE_PLAN` includes the same vertex snapshot and controller mode. `STEP_REFUSED` / `MOVE_FAILED` retain the final stop reason.

The live overlay marks the vertex with a cyan square and draws the outgoing leg. It displays the current vertex state. The overlay consumes the saved plan and does not run perception in the preview thread.

## Verification and next physical test

Run headless tests from the repository root with the normal dependencies installed:

```bash
python -m unittest discover -s tests -v
```

Tests cover sharp turns in both directions and rotated views, concave boundaries, smooth radii, small edge defects, fixed-target persistence, fresh-frame confirmation, delayed-frame poses, four successive corners with a smaller physical step cap, boundary loss, the unchanged 40-pixel gate, Line mode, reset, CSV field retention, and the physical step loop with a mock executor (success and failure).

The October 17 timeline establishes the prior failure but contains no camera frames, so it cannot by itself validate the new geometry fitting. Physical-table validation remains pending.

After pulling this branch, restart FabScan and verify **M6.4** in the controls. Keep the previous test settings and shape for comparison. Use Dry Plan, then test the same first and second corners. A successful transition should show `COMMITTED → VERTEX_REACHED → OUTGOING_LATCHED → RESUMED`. Send the timeline; if geometry is rejected or support is lost, also preserve the automatically recorded frame package for diagnosis.
