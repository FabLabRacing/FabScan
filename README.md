# FabScan v0.3.5 - Assisted Trace Tools

FabScan v0.3.5 adds basic helper tools for cleaning up manually captured CNC trace geometry.

## What this version changes

Adds an **Assisted Trace Tools** section to the LinuxCNC / Manual Trace panel:

- `Line Endpoints` — reduces the active trace to a straight line from its first point to its last point.
- `Rect 2 Pts` — turns the first two active trace points into an axis-aligned rectangle using those points as opposite corners.
- `Circle Fit` — fits a circle through the active trace points and replaces the active trace with sampled circle points.
- `Arc Last 3` — replaces the last three points in the active trace with a sampled arc through those points.
- `Curve pts` — controls the detail used for generated circles and arcs.

These tools modify the **active trace only**. Other trace groups are left alone.

## Why this exists

Manual CNC tracing is useful, but raw point chains are not always the clean geometry you want in CAD/SheetCam.

Examples:

- Touch two opposite corners and use `Rect 2 Pts` for a square or rectangular profile.
- Touch several points around a hole and use `Circle Fit`.
- Touch the start, midpoint, and end of a radius and use `Arc Last 3`.
- Touch a few points along an edge and use `Line Endpoints` to force it straight.

FabScan still exports simple DXF polylines. SheetCam can continue to do detail reduction and arc fitting after import.

## Notes

- `Rect 2 Pts` and `Circle Fit` automatically enable `Closed` trace mode.
- `Arc Last 3` is intended for radius cleanup inside the active trace. It preserves any points before the last three points.
- If you are exporting an open arc by itself, uncheck `Closed` before exporting.
- Jogging remains guarded X/Y incremental only. No continuous jog, no Z jog, no torch commands, no MDI, and no program start.

## Suggested test

1. Start LinuxCNC/QtPlasmaC normally.
2. Open FabScan and verify LinuxCNC is connected in `MANUAL` mode.
3. Capture two opposite corners of a rectangle.
4. Click `Rect 2 Pts`.
5. Use `Start New`.
6. Capture 3 or more points around a circle/hole.
7. Click `Circle Fit`.
8. Export Manual Trace DXF and import into SheetCam/CAD.
