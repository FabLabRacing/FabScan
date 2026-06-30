# FabScan v0.2.2 - Camera Capture Aids

This update adds camera alignment/orientation tools to the camera capture window.

## Added

- Live crosshair overlay.
- X+ / Y+ axis direction labels.
- Optional grid overlay.
- Camera orientation controls:
  - Rotate 0 / 90 / 180 / 270 degrees.
  - Flip X.
  - Flip Y.
  - Fine rotation from -10.0 to +10.0 degrees.
  - Fine adjustment buttons: -1, -0.1, 0, +0.1, +1 degree.
- Captured PNGs are saved after the orientation transform is applied.
- Preview overlays are not saved into the captured PNG.
- Camera aid settings are saved in `~/.config/fabscan/settings.json`.

## Camera workflow

1. Click **Camera Capture**.
2. Open the camera.
3. Use rotate/flip/fine-rotation until the preview lines up with the intended machine/CAD axes.
4. Use the crosshair and axis labels as a setup aid.
5. Click **Capture Frame**.
6. Continue with the normal FabScan workflow:
   - Threshold / cleanup.
   - Find Contours.
   - Enable/disable contours.
   - Set Scale.
   - Run X/Y Sanity Check.
   - Export DXF.

## Axis convention

In the transformed camera preview:

- `X+` points right.
- `Y+` points up.

That matches the intended DXF/CAD orientation after FabScan exports the geometry.

## Notes

Fine rotation is only for small camera-mount alignment errors. It helps make the image square to the desired X/Y axes, but it does not correct camera perspective or lens distortion.

For best results, keep the camera as square to the part/background as possible and use even lighting.
