# FabScan Ver. 0.1 - First Swing

FabScan is a small desktop utility for turning a photo/scan of a flat part into a basic DXF outline that can be cleaned up in SheetCam.

This is intentionally simple and rough. The first goal is to prove the workflow:

1. Load a PNG/JPG image.
2. Threshold it to black/white.
3. Find contours.
4. Set scale using two clicked points and a known distance.
5. Export a DXF.
6. Import the DXF into SheetCam and let SheetCam clean it up.

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

## Basic workflow

1. Click **Load Image**.
2. Adjust **Threshold**, **Blur**, **Min Area**, and **Simplify** until the preview looks reasonable.
3. Click **Find Contours**.
4. Click **Set Scale**.
5. Click two known points on the image.
6. Enter the real-world distance between those points in inches.
7. Click **Export DXF**.

## Notes

- DXF units are inches.
- OUTSIDE contours go on the `OUTSIDE` layer.
- Hole/internal contours go on the `INSIDE` layer when OpenCV hierarchy can identify them.
- The Y axis is flipped on export so the DXF opens in a normal CAD/CAM orientation instead of image coordinates.
- This version does not try to fit arcs. SheetCam can handle detail reduction, arc fitting, circle recognition, and duplicate line cleanup.
