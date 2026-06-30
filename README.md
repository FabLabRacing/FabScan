# FabScan v0.3.0 - LinuxCNC Manual Trace

This update keeps the working camera/image-to-DXF workflow and adds the first machine-side tracing feature.

## Added

- **LinuxCNC / Manual Trace** side-panel.
- Read-only LinuxCNC status polling.
- Current X/Y/Z position display.
- Coordinate source selector:
  - Work coordinates
  - Machine coordinates
- Manual point capture while jogging normally in LinuxCNC/QtPlasmaC.
- Undo / clear captured points.
- Closed/open trace option.
- Export manually captured points as a DXF on layer `TRACE`.
- Optional auto-refresh for LinuxCNC position display.

## Safety boundary

FabScan v0.3.0 is read-only with LinuxCNC.

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
4. In **LinuxCNC / Manual Trace**, click **Refresh Position**.
5. Choose **Work coordinates** or **Machine coordinates**.
6. Jog the machine/pointer to the first feature point.
7. Click **Capture Point**.
8. Repeat around the profile.
9. Leave **Closed** checked for a closed part outline, or uncheck it for an open reference trace.
10. Click **Export Manual Trace DXF**.
11. Verify/import the DXF in SheetCam or CAD.

## Existing image/camera workflow

1. Load Image or use Camera Capture.
2. Adjust Threshold / Blur / Noise Removal / Edge Cleanup.
3. Click Find Contours.
4. Enable/disable contours so only wanted geometry exports.
5. Click Set Scale, pick two known points, and enter the real distance.
6. Use the X/Y Sanity Check against known CNC/part dimensions.
7. Export DXF and bring it into SheetCam/CAD for final cleanup.

## Notes

- Manual trace DXF points are exported exactly as captured in X/Y.
- Z is displayed and stored in the trace list for reference, but DXF export is 2D.
- The DXF layer for manual trace output is `TRACE`.
- FabScan sets DXF units to inches.
- If the `linuxcnc` Python module is not available in the virtual environment, FabScan still opens; the LinuxCNC panel will report that the module is missing.
