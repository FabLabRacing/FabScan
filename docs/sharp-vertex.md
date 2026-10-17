# M6.4 sharp-vertex commitment

Implementation identifier: `0.6.0-alpha.1-m6.4.3`. The camera controls and overlay show **M6.4**.

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
- Every committed step rechecks the frozen corner transition and the untraversed straight boundary, with a maximum **4-pixel** residual outside the central **0.015 inch**. APPROACH checks the entire remaining incoming leg (not just 0.080 inch), with a 0.003-inch overlap behind the actual camera position, plus 0.080 inch of outgoing support. OUTGOING retires incoming support and checks the outgoing segment from actual progress minus that overlap through 0.080 inch beyond the vertex. Samples are at most 0.005 inch apart. The corner transition remains checked in both states using the bounded connected-region check below.
- Support retirement follows the actual controller pose, never a proposed endpoint. A defect behind that overlap cannot veto the next move; missing support on the remaining path still produces HOLD.
- Inside the central 0.015 inch, the fixed target is an **ideal fitted intersection**, not necessarily a literal contour pixel. The current observed boundary must connect the two predicted leg endpoints (each within 4 pixels) **entirely inside a 0.018-inch radius** of that intersection. Use the actual one-pixel contour with eight-neighbor connectivity: no dilation, filling or gap bridging. The radius uses calibrated machine distance, including skew/unequal pixel scales.
- This explicitly permits a small localized tip irregularity relative to the ideal corner. It does **not** guarantee that motion through the fitted intersection stays on every raw contour pixel. The endpoint and target are never moved to chase that irregularity. Larger changes, disconnected support, shifted legs and a neighborhood outside the view still HOLD. Initial two-frame sharp-vertex credibility checks remain unchanged; this does not enable initial commitment to general rounded contours.
- During commitment, boundary association uses the frame-pose projection onto the frozen incoming/outgoing leg. It does not use the detector's blended corner location or change the frozen target. The current camera must stay within the original commitment corridor and leg extent.

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

The supplied frame packages permit saved-decision replay. They end at their respective HOLDs, so they cannot establish successful physical execution beyond those points. Physical-table validation of the latest patch remains pending.

After pulling this branch, restart FabScan and verify **M6.4** in the controls. Keep the previous test settings and shape for comparison. Use Dry Plan, then test the same first and second corners. A successful transition should show `COMMITTED → VERTEX_REACHED → OUTGOING_LATCHED → RESUMED`. Send the timeline; if geometry is rejected or support is lost, also preserve the automatically recorded frame package for diagnosis.

## M6.4.1 recording regression

The October 17 13:45:35 recording stops at step 93 in M6.4.0: detector seed distance 40.42 pixels. Replaying saved steps 88–93 with the recorded calibration and fine rotation reproduces that HOLD. M6.4.1 accepts all six saved approach decisions, with step 93 targeting the original vertex in a 0.027077-inch move. Its committed anchor is 1.68 pixels from the boundary; required incoming/outgoing residuals are 2.87/2.32 pixels. The old 4.65-pixel incoming sample is behind the actual camera and outside the retained overlap.

This is a saved-frame planning regression, not evidence of a completed second corner on the machine: the recording ends before that move. Automated tests also cover detector drift with delayed frame pose, displaced boundaries, defects ahead versus behind, outgoing support loss, and CSV persistence of association source, support ranges and residuals.

## M6.4.2 tip support regression

The 14:57:37 recording confirms a sharp corner at step 14, then holds at step 18, 0.042477 inch short of the target. The committed boundary association is only 0.50 pixels from the contour. The maximum 6.43-pixel distance-transform residual is at the ideal intersection, not a traversed incoming defect. Samples farther along the two straight legs remain within the 4-pixel bound. Leg analysis does not establish a coherent translation that would justify moving the frozen target; the mismatch is concentrated at the tip.

M6.4.2 checks that tip as the bounded connected transition described above. All 17 committed decisions in both supplied frame packages pass replay using saved calibration, fine rotation and poses: 12 from the earlier recording, 5 from the new one. Step 18 now plans the next bounded 0.030-inch approach move toward the original vertex; required incoming/outgoing straight support residuals are 2.87/0.95 pixels. It does not claim arrival at the vertex from that frame. The earlier step 93 still plans its final 0.027077-inch move.

New timeline columns: `vertex_corner_radius`, `vertex_corner_connected` and `vertex_tip_distance_px` (Euclidean pixel distance, which differs slightly from OpenCV's approximate distance-transform residual). Tests cover a localized changed tip through executed arrival/outgoing release without retargeting, a tip change outside the envelope, a disconnected transition and empty local support. Existing straight-leg loss, boundary gate, delayed-pose, smooth-radius and executor-state tests remain active.

## M6.4.3 estimator handoff

During APPROACH/OUTGOING, the live estimator now consumes a fresh observation of the same committed boundary used by the planner. Before an observation is admitted, it passes the connected-component, Edge representation, remaining straight-leg support, local corner connectivity, pose corridor and detector-confidence checks. The observation measures the nearest contour's normal offset on a supported straight segment outside the tip neighborhood. Its along-leg station uses the frame pose; its direction uses the confirmed, revalidated leg. It is not a free fit of both legs together.

The first observation on a newly committed incoming leg starts a new filter history. When a successful executed pose transitions APPROACH to OUTGOING, the runtime retires the incoming measurement and curvature histories and marks the filter for reinitialization from the next valid outgoing observation. Diagnostic state records remain available. Planning, Dry Plan and failed execution cannot trigger that physical outgoing handoff. Repeated observations on the same leg do not keep resetting the filter. Once the outgoing exit target is reached, ordinary detector observations resume with the outgoing estimator state as history.

The current detector still has to find a profile and pass its confidence gate. Its confidence is carried into the boundary observation; no confidence floor is added. Invalid geometry produces an invalid belief with zero confidence, rather than falling back to the ambiguous detector point or a stored target. The motion planner independently rechecks the frame before approving motion. Confidence thresholds, fixed targets, support bounds and the LinuxCNC executor remain unchanged.

The 15:16:06 recording reached the first vertex, latched outgoing travel and moved 0.030 inch, then held with ordinary belief confidence 42.895% against the 45% minimum. The belief heading remained near 159 degrees while the outgoing leg was 93.34 degrees. A sequential replay through the real estimator, live belief builder and planner now uses a 93.34-degree outgoing estimate; at the final frame its belief/planner confidence is about 68.37/66.32%. It plans the remaining approximately 0.030006-inch move to the fixed exit target.

All 135 recorded decisions across the three October 17 packages pass the estimator/planner replay (93 + 18 + 24). Replay uses saved images, calibration, fine rotation and actual poses; stabilized detector measurements are reconstructed from the recorded belief measurements and innovations. Actual recorded poses remain the inputs, so this is not a closed-loop simulation of alternate physical motion. No frames after the final HOLDs exist.

Tests now exercise the real estimator through commitment, executed arrival, outgoing travel and return to ordinary following, including delayed poses, rotated views, incoming-biased generic measurements, invalid/lost support, low detector confidence and observed boundary displacement. The physical-step test also runs with the real belief builder/estimator and a mock executor for both success and failure. New CSV fields are `belief_measurement_source` and `belief_history_reset` on `M6_BELIEF` events. `committed_incoming`/`committed_outgoing` identify admitted boundary observations; rejected observations are logged separately. The controls still show M6.4; use the full planner version or commit to distinguish patches.
