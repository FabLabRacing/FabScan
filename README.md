# FabScan v0.3.2 - Multi-Trace Manual Capture

This update keeps the working camera/image-to-DXF workflow and improves the LinuxCNC manual trace workflow by allowing multiple separate traced contours.

## Added

- **Start New** button in the LinuxCNC / Manual Trace panel.
- Multiple manual trace groups/contours.
- Trace point list now shows trace number and point number.
- Trace preview draws each trace group separately, without connecting one contour to the next.
- Manual trace DXF export writes each trace group as a separate DXF polyline on layer `TRACE`.

## Why this matters

This lets you trace shapes like:

- square inside a square
- outside profile plus hole
- outside profile plus slot
- multiple disconnected reference shapes

Use **Start New** after finishing one contour, then capture points for the next contour.

## Safety boundary

FabScan v0.3.2 is still read-only with LinuxCNC.

It does **not**:

- jog the machine
- send MDI commands
- move Z
- fire the torch
- start a program
- change LinuxCNC state

Use LinuxCNC/QtPlasmaC to jog the table like normal, then capture points in FabScan.

## Manual CNC trace workflow

1. Start LinuxCNC/QtPlasmaC normally.
2. Home the machine and set work zero if you want the trace in work coordinates.
3. Open FabScan.
4. Turn on **Auto** refresh or click **Refresh Position**.
5. Jog the machine in LinuxCNC/QtPlasmaC.
6. Click **Capture Point** at each traced location.
7. When one contour is done, click **Start New**.
8. Capture points for the next contour.
9. Choose **Closed** or open trace.
10. Click **Export Manual Trace DXF**.

The manual trace DXF exports on layer `TRACE` and uses the captured CNC X/Y coordinates directly.

## Trace preview notes

- `X+` points right.
- `Y+` points up.
- Active trace points are yellow with green connecting lines.
- Previous trace groups are drawn separately in blue.
- Point labels use `trace.point` format, such as `1.4` or `2.1`.
- The red crosshair marks the latest live LinuxCNC position when enabled.
- Uncheck **Trace Preview** to return the main canvas to image preview when image tracing.
