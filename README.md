# FabScan v0.4.0 - Controlled X/Y Motion

FabScan v0.4.0 adds the first controlled motion workflow for LinuxCNC.

This is intentionally conservative:

- X/Y only
- one point-to-point move at a time
- G1 feed move, not rapid-only motion
- no Z motion
- no torch/plasma commands
- no program start
- no automatic trace following
- explicit enable checkbox required each session
- confirmation dialog before each controlled move
- STOP Move button sends a LinuxCNC abort command

## Controlled Motion panel

The new panel is named:

```text
Controlled Motion - X/Y Point Move
```

It includes:

```text
Enable controlled moves
Target X
Target Y
Feed/min
Use Current
Use Selected Pt
Move to Target
STOP Move
```

## Basic workflow

1. Start LinuxCNC/QtPlasmaC normally.
2. Home the machine.
3. Keep the torch/plasma disabled while testing.
4. Refresh LinuxCNC in FabScan.
5. Enable controlled moves.
6. Set a small feed/min value.
7. Set a target using either:
   - `Use Current`, then edit X/Y manually, or
   - select a captured trace point and click `Use Selected Pt`.
8. Click `Move to Target`.
9. Confirm the move.
10. Use the physical E-stop if anything unexpected happens. `STOP Move` is only a software abort.

## Coordinate behavior

Controlled moves use the current FabScan coordinate source:

- `Work coordinates` sends a normal absolute work-coordinate `G90 G1 X... Y... F...` move.
- `Machine coordinates` sends a machine-coordinate `G90 G53 G1 X... Y... F...` move.

For normal tracing, `Work coordinates` is usually the preferred mode.

## Safety checks

FabScan refuses controlled moves unless LinuxCNC is:

- connected
- task state ON
- interpreter IDLE
- task mode MANUAL or MDI
- X/Y/Z homed

FabScan also limits controlled move feed to 120 units/minute.

## Notes

This version does not follow an entire contour automatically. It only moves to one X/Y target at a time.

The next likely step is an assisted point-to-point workflow, such as `Move to Next Trace Point`, after this version is proven stable.
