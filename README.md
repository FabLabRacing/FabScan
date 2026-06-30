# FabScan v0.2.3 - Polish / Stability

This update keeps the working v0.2.2 camera/DXF workflow and adds small usability improvements.

## Added

- Version number in the main window title.
- **Help > Basic Workflow** menu item.
- **Help > About FabScan** menu item.
- Toolbar **Help** button.
- Toolbar/menu **Reset Recommended Defaults** action.
- Better status text after loading/capturing an image.
- Better DXF export summary after saving a DXF.

## Reset Recommended Defaults

The reset action restores the main tracing/export controls:

- Threshold
- Blur
- Noise Removal
- Edge Cleanup
- Min Area
- Simplify %
- Invert
- Show Threshold
- X/Y sanity check fields
- DXF origin/margin
- Contour list show/sort

It intentionally keeps camera orientation/settings, camera size, last folders, and window position.

## Basic workflow

1. Load Image or use Camera Capture.
2. Adjust Threshold / Blur / Noise Removal / Edge Cleanup.
3. Click Find Contours.
4. Enable/disable contours so only wanted geometry exports.
5. Click Set Scale, pick two known points, and enter the real distance.
6. Use the X/Y Sanity Check against known CNC/part dimensions.
7. Export DXF and bring it into SheetCam/CAD for final cleanup.

## Notes

- Disabled contours stay visible in gray but do not export.
- Use **Show Threshold** to see what FabScan is actually tracing.
- Keep cleanup values low unless the camera image is ugly.
- X+ is right and Y+ is up in the transformed camera preview.
