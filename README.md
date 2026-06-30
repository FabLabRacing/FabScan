# FabScan v0.3.4 - Jog Mode Cleanup

FabScan v0.3.4 cleans up the guarded X/Y incremental jog behavior added in v0.3.3.

## What this version changes

- Removes the forced LinuxCNC task-mode change before each jog.
- Adds a task-mode display to the LinuxCNC / Manual Trace panel.
- Refuses FabScan jogs unless LinuxCNC is already in `MANUAL` mode.
- Adds a short jog-busy lockout after successful step jogs.
- Improves jog status messages.

## Why this patch exists

If FabScan asks LinuxCNC to switch task mode while an incremental jog is still active, LinuxCNC can report:

```text
Ignoring task mode change while jogging
```

FabScan should not be changing task modes during every jog. This version treats task mode as a required precondition instead:

1. Put LinuxCNC/QtPlasmaC in manual/jog mode.
2. Enable FabScan jog controls.
3. Use FabScan X/Y step jog buttons.

## Jog safety behavior

FabScan will refuse to jog unless:

- LinuxCNC is connected.
- LinuxCNC task state is ON.
- LinuxCNC interpreter state is IDLE.
- LinuxCNC task mode is MANUAL.
- X/Y/Z show as homed.
- Jog controls were explicitly enabled for this session.
- The requested jog is X or Y only.
- The requested step/feed are inside FabScan's conservative limits.

FabScan v0.3.4 still does not provide continuous jog, Z jog, torch commands, MDI moves, or program start.

## Manual trace workflow

1. Start LinuxCNC/QtPlasmaC normally.
2. Home the machine and disable the torch/plasma output for tracing.
3. Put LinuxCNC/QtPlasmaC in manual/jog mode.
4. Open FabScan.
5. Refresh LinuxCNC status and verify task mode shows `MANUAL`.
6. Enable FabScan jog controls if you want to jog from FabScan.
7. Use X/Y step jog buttons to move to a trace point.
8. Click `Capture Point`.
9. Use `Start New` for another contour, such as an inside cutout.
10. Export the manual trace DXF.
