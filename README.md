# FabScan v0.4.1 - Point / Trace Navigation

FabScan v0.4.1 adds point navigation and trace editing tools on top of the v0.4.0 controlled X/Y motion workflow.

## Added

- Point / Trace Navigation panel.
- First Pt / Prev Pt / Next Pt / Last Pt selection buttons.
- Selecting a point makes that trace the active trace.
- Selected point is highlighted in the Manual Trace Preview.
- Move Pt button copies the selected point to the controlled-motion target and uses the existing guarded Move to Target workflow.
- Replace button replaces the selected captured point with the current LinuxCNC position.
- Insert After button inserts the current LinuxCNC position after the selected point.
- Delete Pt button removes the selected point and keeps trace groups valid.

## Safety behavior

Motion behavior is unchanged from v0.4.0:

- X/Y only.
- No Z motion.
- No torch commands.
- No program start.
- Controlled moves require explicit enable.
- Each controlled move still asks for confirmation.
- LinuxCNC state checks are still handled by the LinuxCNC adapter before motion.

## Typical correction workflow

1. Capture a trace.
2. Select a point in the trace list or use Prev/Next.
3. Click Move Pt to return to that point.
4. Jog/fine move to the corrected location.
5. Click Replace to update that point.
6. Export Manual Trace DXF.

## Install drop-in files

```bash
cd ~/projects/FabScan
cp /path/to/unzipped/FabScan_v041_point_navigation_dropin/fabscan/app.py fabscan/app.py
cp /path/to/unzipped/FabScan_v041_point_navigation_dropin/README_v0.4.1.md README.md
```

Then test:

```bash
source .venv/bin/activate
python3 -m py_compile fabscan/*.py fabscan.py
python3 fabscan.py
```
