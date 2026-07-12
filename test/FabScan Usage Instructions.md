# FabScan Usage Instructions

FabScan is a LinuxCNC companion utility for capturing usable 2D geometry from images, a USB camera, or manual CNC position points, then exporting DXF for cleanup/CAM.

FabScan is intended to be a **tracing/scanning helper**, not a full CAD/CAM package. A normal workflow is:

```text
part / image / camera view -> FabScan -> DXF -> SheetCam / CAD cleanup -> CAM output
```

FabScan can be used in three main ways:

1. **Image contour tracing**
   Load a photo or captured image, detect contours, set scale, and export DXF.

2. **Manual CNC point tracing**
   Use LinuxCNC position as the measuring reference, jog around a real part, capture points, and export DXF.

3. **Camera calibration and line/edge following**
   Use a USB camera and LinuxCNC X/Y motion to calibrate camera-to-machine direction, then follow a detected line or edge in small guarded steps.

FabScan is intentionally limited to **X/Y motion only**. It does not control Z, torch firing, plasma start, cycle start, spindle, THC, or any cutting output.

---


# 1. Starting FabScan

From the FabScan project folder:

```bash
python3 fabscan.py
```

If running from a virtual environment, install the Python requirements first:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 fabscan.py
```

On a LinuxCNC machine, start LinuxCNC normally first if you plan to use CNC position, jog, controlled move, calibration, or follow features.

---


# 2. Image Contour Tracing Workflow

This is the simplest FabScan workflow and does not require LinuxCNC motion.

## Step 1 - Load or capture an image

Use one of the toolbar buttons:

```text
Load Image
```

or:

```text
Camera Capture
```

For best results, use a high-contrast image where the part is clearly separated from the background.

Good images trace better than bad images. A clean silhouette, dark marker line, or high-contrast edge will usually work better than a cluttered photo.

## Step 2 - Adjust cleanup settings

Use the **Image Cleanup** controls:

```text
Threshold
Blur
Noise Removal
Edge Cleanup
Min Area
Simplify %
```

Suggested starting point:

```text
Threshold: adjust until the part/line separates clearly
Blur: 3
Noise Removal: 0
Edge Cleanup: 0
Min Area: 1000
Simplify %: 0.05
```

Use:

```text
Show Threshold
```

to see what FabScan is actually detecting.

Use:

```text
Invert
```

if the part/background are backwards.

A good rule is to keep cleanup settings low at first. Higher cleanup values can make the image look nicer, but they can also change real geometry.

## Step 3 - Find contours

Click:

```text
Find Contours
```

FabScan will detect possible geometry and list the contours in the **Contours** panel.

Contours are displayed in the preview window and listed with ID, enabled state, layer/classification, area, and point count.

## Step 4 - Enable only the geometry you want

Use the contour list and buttons:

```text
Toggle Selected
Enable All
Disable All
Enable Visible
Disable Visible
```

Disabled contours stay visible in gray but do not export.

Use the filter/sort controls to make the list easier to work with:

```text
Show: All / Enabled only / Disabled only / OUTSIDE / INSIDE
Sort: Layer + area / Area largest first / Area smallest first / ID / Points most first
```

## Step 5 - Set scale

Click:

```text
Set Scale
```

Then click two known points in the image and enter the real-world distance between them.

The scale controls the DXF size. If the scale is wrong, the exported DXF will be wrong.

After setting scale, use the **Measurements** panel to check the detected size.

## Step 6 - Use X/Y Sanity Check

If you know the real part size, enter:

```text
Expected W in
Expected H in
Tol +/- in
```

This compares the enabled contour bounding box against the expected dimensions.

The sanity check does not change scale. It is only a warning/checking tool.

## Step 7 - Export DXF

Choose the DXF export origin:

```text
Move lower-left to 0,0
Preserve image position
Center on 0,0
```

For SheetCam use, the simplest starting point is usually:

```text
Move lower-left to 0,0
```

Then click:

```text
Export DXF
```

Bring the DXF into SheetCam or CAD for final cleanup/tooling.

---

# 3. Camera Capture Workflow

Click:

```text
Camera Capture
```

The camera capture window lets you preview the USB camera and capture a still frame into the main tracing window.

Suggested current camera settings:

```text
Camera index: 0
Resolution: 800 x 600
```

Available resolution presets include:

```text
800 x 600
640 x 480
1280 x 960
1600 x 1200
1280 x 720
```

Use:

```text
Open / Restart Camera
```

to open the camera after changing index or resolution.

Use:

```text
Capture Frame
```

to send the current frame back to the main FabScan window.

The camera status line shows the requested resolution, actual resolution, backend, and camera format. If the requested and actual resolutions do not match, the camera or driver may not support the requested mode.


If the camera opens but no frames appear, try:

```text
640 x 480
800 x 600
```

and try another camera index. Some Linux camera devices expose metadata nodes that OpenCV can open but cannot actually read frames from.

---

# 4. Manual CNC Point Trace Workflow

Manual CNC tracing uses LinuxCNC position as the measuring reference.

This is useful when you want to jog around a real part and capture actual machine coordinates.

## Step 1 - Prepare LinuxCNC

In LinuxCNC / QtPlasmaC:

```text
Reset E-stop
Machine ON
Home all axes
Switch to MANUAL mode
Disable torch/plasma output
```

## Step 2 - Refresh LinuxCNC position

In FabScan, click:

```text
Refresh LinuxCNC
```

or in the trace panel:

```text
Refresh Position
```

FabScan should show:

```text
Status: Connected
State: ON / IDLE
Task mode: MANUAL
Homed: X:Y Y:Y Z:Y
```

Choose the coordinate source:

```text
Work coordinates
```

or:

```text
Machine coordinates
```

For most part-tracing work, **Work coordinates** is usually the practical choice.

## Step 3 - Capture points

Jog the machine to the first point and click:

```text
Capture Point
```

Jog to the next point and click:

```text
Capture Point
```

Repeat until the contour is captured.

Use:

```text
Start New
```

when beginning a separate contour, such as a hole inside an outside profile.

Use:

```text
Undo
Clear
```

as needed.

The **Trace Preview** shows the captured geometry.

## Step 4 - Closed or open trace

Use the:

```text
Closed
```

checkbox to control whether the trace should be treated as a closed contour.

For an outside profile or hole, use **Closed**.

For an open line, leave **Closed** unchecked.

## Step 5 - Edit points if needed

The point navigation tools include:

```text
First Pt
Prev Pt
Next Pt
Last Pt
Move Pt
Replace
Insert After
Delete Pt
```

`Move Pt` uses the controlled-motion path to move back to a selected captured point.

`Replace` replaces the selected point with the current LinuxCNC position.

`Insert After` inserts the current LinuxCNC position after the selected point.

`Delete Pt` removes the selected point.

## Step 6 - Assisted trace tools

FabScan can turn captured points into cleaner native DXF geometry using:

```text
Line Endpoints
Rect 2 Pts
Circle Fit
3 Pt Arc
Set Center
Center Arc
Clear Center
```

The circle and arc tools export native DXF geometry instead of only segmented polylines.

Basic examples:

```text
Line Endpoints
```

Use when the first and last captured points define a straight line.

```text
Rect 2 Pts
```

Use when two points define opposite corners of a rectangle.

```text
Circle Fit
```

Use when several captured points lie around a circle.

```text
3 Pt Arc
```

Use exactly three points to define an arc.

```text
Set Center
Center Arc
```

Use when you know/capture the arc center, then capture two arc endpoints.

## Step 7 - Export manual trace DXF

Click:

```text
Export Manual Trace DXF
```

Then bring the DXF into SheetCam or CAD for cleanup/tooling.

---

# 5. FabScan Jog Controls

FabScan has simple X/Y incremental jog buttons.

This is not continuous jogging. Each click commands one guarded incremental X/Y step.

To use it:

1. Make sure LinuxCNC is ON, IDLE, MANUAL, and homed.
2. Check:

```text
Enable jog controls
```

3. Set:

```text
Step
Feed/min
```

4. Use:

```text
X+
X-
Y+
Y-
```

FabScan limits jog step and feed to conservative values.

Current jog behavior:

* X/Y only
* No Z
* No torch
* No continuous jog
* LinuxCNC must already be in MANUAL mode
* LinuxCNC must be homed
* LinuxCNC interpreter must be IDLE

---

# 6. Controlled X/Y Point Move

The controlled motion section can command one point-to-point X/Y move.

To use it:

1. Make sure LinuxCNC is ON, IDLE, homed, and in MANUAL or MDI mode.
2. Check:

```text
Enable controlled moves
```

3. Enter:

```text
Target X
Target Y
Feed/min
```

or use:

```text
Use Current
Use Selected Pt
```

4. Click:

```text
Move to Target
```

FabScan will show a confirmation before commanding the move.

Use:

```text
STOP Move
```

to send an abort command through LinuxCNC.

The physical E-stop is still the real safety device. `STOP Move` is a software abort, not a replacement for E-stop.

Controlled move behavior:

* X/Y only
* No Z
* No torch
* One explicit G1 move
* Feed-limited
* LinuxCNC state checked before motion

---

# 7. Camera Calibration Lite

Camera Calibration Lite teaches FabScan how camera pixels relate to machine X/Y motion.

This is required before using camera-based dot centering or line/edge following.

Click:

```text
Camera Calibrate
```

## Step 1 - Prepare LinuxCNC

LinuxCNC must be:

```text
ON
IDLE
MANUAL mode
Homed
```

The torch/plasma should be disabled.

## Step 2 - Open the camera

Start with:

```text
Camera index: 0
Resolution: 800 x 600
```

Click:

```text
Open / Restart
```

If there is no live image, try another resolution or camera index.

## Step 3 - Set camera orientation

Use:

```text
Rotate
Flip X
Flip Y
Fine
```

to make the preview match the machine as closely as practical.

The goal is that X+ and Y+ make sense visually in the transformed preview.

## Step 4 - Detect the calibration dot

Place a clear dot/mark under the camera.

Adjust:

```text
Threshold
```

until the dot is detected cleanly.

Click:

```text
Find Dot
```

FabScan should report the dot location.

Use the small calibration jog controls if needed:

```text
X-
X+
Y+
Y-
```

to put the dot somewhere reasonable in the view.

## Step 5 - Run calibration

Set:

```text
Move
Feed
```

A typical starting value is:

```text
Move: 0.100
Feed: 5.0 units/min
```

Click:

```text
Run Calibration
```

FabScan will:

1. Detect the starting dot.
2. Jog X+ by the calibration move distance.
3. Detect the dot again.
4. Return to the start point.
5. Jog Y+ by the calibration move distance.
6. Detect the dot again.
7. Return to the start point.
8. Build a camera-to-machine calibration matrix.

When calibration succeeds, FabScan reports the calibration result and saves it in settings.

## Step 6 - Center Dot

After calibration, use:

```text
Center Dot
```

to make FabScan calculate a small X/Y correction to move the detected dot toward the camera crosshair.

Use:

```text
Max center
```

to limit the size of this correction.

This is useful for checking that calibration direction and scale make sense before trying line/edge following.

---

# 8. Line / Edge Preview

The line/edge preview tools are inside Camera Calibration Lite.

They are used to detect a line, edge, or line center in the camera image.

Main controls:

```text
Mode
Search px
Overlay
Find Line / Edge
```

Typical current test mode:

```text
Mode: Line center
Search px: 220
Overlay: enabled
```

Click:

```text
Find Line / Edge
```

FabScan will report whether it found the target, the pixel offset from center, and the detection confidence.

The line/edge overlay lets you see what FabScan thinks it is following.

If the wrong thing is detected, improve the visual target, adjust lighting, adjust the threshold/mode/search area, or reposition the camera.

---

# 9. Single-Step Line / Edge Following

Line/edge following is still experimental, but the current proof of concept is usable for cautious testing.

Before following:

1. Run camera calibration successfully.
2. Verify dot centering behaves correctly.
3. Put the line/edge under the camera.
4. Click:

```text
Find Line / Edge
```

5. Check:

```text
Enable follow
```

6. Start with conservative settings.

Suggested starting settings:

```text
Step: 0.050
Max correct: 0.010 to 0.015
Min conf: 55 to 60
Count: 5 to 10 for early testing
Direction: Forward or Reverse
Capture after move: optional
```

Click:

```text
Follow Step
```

FabScan will:

1. Detect the line/edge.
2. Convert the camera-space line direction into machine-space motion.
3. Move a small distance along the line.
4. Apply a limited side correction toward the line/edge.
5. Re-check the camera image after the move.

If the first step goes the wrong direction, change:

```text
Direction
```

then click:

```text
Find Line / Edge
```

again before continuing.

The first follow step uses the selected Forward/Reverse direction. Later steps latch to the previous successful machine heading so the tangent direction does not randomly flip 180 degrees.

---

# 10. Multi-Step Follow

After single-step follow works, use:

```text
Follow N
```

FabScan will run the requested number of bounded follow steps.

This is not continuous free-running motion. It is repeated small moves with checks between steps.

FabScan stops if:

* The line/edge is not found.
* Confidence drops below the minimum.
* LinuxCNC is not ready.
* Motion is already active.
* STOP Move is pressed.
* The calculated move is invalid or too small.

Use:

```text
STOP Move
```

to stop/abort motion.

For early testing, use a small count:

```text
Count: 5
```

Then increase gradually:

```text
Count: 10
Count: 25
Count: 50
```

Only increase count after the line/edge behavior is predictable.

For tight curves or 180-degree bends, reduce step size and keep correction conservative.

---

# 11. Capturing Points While Following

In the follow tools, use:

```text
Capture after move
```

when you want each successful follow move to add a point to the active manual trace.

A useful workflow is:

1. Run calibration.
2. Find the line/edge.
3. Enable follow.
4. Enable Capture after move.
5. Run Follow Step or Follow N.
6. Review captured points in the manual trace preview.
7. Use assisted trace tools if needed.
8. Export Manual Trace DXF.

This is the closest current workflow to camera-assisted CNC scanning.

---

# 12. Recommended Current Camera Settings

For the current tested USB microscope camera, start with:

```text
Camera index: 0
Resolution: 800 x 600
```

If that fails, try:

```text
640 x 480
```

Avoid:

```text
1280 x 720
```

unless the camera status confirms it is actually supported.

Some cameras support `1280 x 960` or `1600 x 1200`, but larger frames can be slower and are not necessarily better for tracing.

---

# 13. Current Recommended Follow Settings

Good starting point from current testing:

```text
Mode: Line center
Step: 0.050
Max correct: 0.010 to 0.015
Min conf: 55 to 60
Count: 10 to 50
Camera: 800 x 600
```

For cautious first tests:

```text
Step: 0.025
Max correct: 0.005 to 0.010
Min conf: 60
Count: 5
```

For straighter, cleaner lines after confidence improves:

```text
Step: 0.050
Max correct: 0.010 to 0.015
Min conf: 55
Count: 25 to 50
```

---


# 14. Settings File

FabScan saves user settings automatically.

On Linux, settings are normally stored at:

```text
~/.config/fabscan/settings.json
```

On Windows, settings are stored under the user AppData config path.

Settings include camera index, camera resolution, image cleanup values, trace options, jog/feed values, and camera calibration data.

Use:

```text
Reset Defaults
```

to reset the main tracing/export controls to recommended defaults. Camera orientation, camera size, last folders, and window position are preserved.

---

# 15. Troubleshooting

## Camera opens but no image appears

Try:

```text
640 x 480
800 x 600
```

Try another camera index.

The selected camera index may be a Linux metadata device rather than the real video stream.

## Camera is slow or FabScan feels sluggish

Use a lower resolution.

Start with:

```text
800 x 600
```

or:

```text
640 x 480
```

Avoid unsupported modes.

## Contours are wrong

Try:

```text
Show Threshold
Invert
Threshold adjustment
Lower cleanup values
Better lighting
Cleaner background
```

Make sure the part or line is visually separated from the background.

## DXF size is wrong

Set scale again.

The most common cause is incorrect scale distance or clicking the wrong two scale points.

## LinuxCNC position does not show

Click:

```text
Refresh LinuxCNC
```

Make sure FabScan is running on the LinuxCNC machine or in an environment where the LinuxCNC Python module and status channel are available.

## Jog or calibration is refused

Check LinuxCNC state.

Required for jog/calibration/follow:

```text
Machine ON
Interpreter IDLE
Task mode MANUAL
X/Y/Z homed
```

## Controlled move is refused

Controlled moves require:

```text
Machine ON
Interpreter IDLE
Task mode MANUAL or MDI
X/Y/Z homed
```

## Follow goes the wrong way on the first step

Change:

```text
Direction: Forward / Reverse
```

Then click:

```text
Find Line / Edge
```

again before following.

## Follow loses the line

Use smaller steps, improve lighting, increase contrast, reduce search confusion, or raise the minimum confidence.

For early testing, use:

```text
Step: 0.025
Count: 5
Min conf: 60
```

---

# 16. Practical First Test

A good first test is not a real part.

Use a piece of flat material with a bold Sharpie line.

1. Start LinuxCNC.
2. Home the machine.
3. Disable torch/plasma output.
4. Open FabScan.
5. Open Camera Calibrate.
6. Open the camera at 800 x 600.
7. Run dot calibration.
8. Verify Center Dot moves the correct way.
9. Put the Sharpie line under the camera.
10. Click Find Line / Edge.
11. Enable follow.
12. Use:

```text
Step: 0.050
Max correct: 0.010
Min conf: 60
Count: 5
```

13. Run Follow Step once.
14. If direction is correct, try Follow N.
15. Enable Capture after move if you want FabScan to collect trace points.

Once that works reliably, move on to real part edges or more complex traces.

---