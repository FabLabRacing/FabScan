# FabScan Ver. 0.1.6 - Contour List Cleanup

FabScan is a small desktop utility for turning a photo/scan of a flat part into a basic DXF outline that can be cleaned up in SheetCam.

This is intentionally simple and rough. The first goal is to prove the workflow:

1. Load a PNG/JPG image.
2. Threshold it to black/white.
3. Find contours.
4. Set scale using two clicked points and a known distance.
5. Enable/disable detected contours as needed.
6. Check selected-contour and enabled-export bounding box measurements.
7. Choose DXF origin behavior and optional lower-left margin.
8. Export a DXF.
9. Import the DXF into SheetCam and let SheetCam clean it up.

## New in v0.1.6

FabScan now has a more useful contour list:

- Show all contours
- Show enabled contours only
- Show disabled contours only
- Show OUTSIDE contours only
- Show INSIDE contours only
- Sort by layer/area, area, ID, or point count
- Enable or disable only the contours currently visible in the filtered list

This makes it easier to deal with noisy images where FabScan finds extra specks,
marks, shadows, or other junk contours.

## Added in v0.1.5

FabScan now remembers the last-used settings between runs:

- Window geometry
- Threshold
- Blur
- Minimum contour area
- Simplify percent
- Invert
- Show Threshold
- DXF origin option
- DXF margin
- Contour list show/sort options
- Last image-open folder
- Last DXF-export folder

The settings file is saved outside the git repo:

```text
Linux: ~/.config/fabscan/settings.json
Windows: %APPDATA%/fabscan/settings.json
```

## Install

From inside the `FabScan` folder:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python3 fabscan.py
```

## Suggested first tests

Use a clean PNG from Inkscape:

- White background
- Black 4" x 4" square
- Export PNG at 300 or 600 DPI
- Use Invert if the part is black on white
- Click two known points and enter the known distance
- Export DXF and verify in SheetCam
