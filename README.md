# FabScan v0.4.3 - Native Trace Continuation

FabScan v0.4.3 fixes a workflow issue from v0.4.2 where adding a new captured point after fitting a native arc/circle/line would clear the native entity and revert the active trace to a polyline.

## What changed

If the active trace has already been fitted to a native DXF entity and you capture another point, FabScan now:

1. Preserves the fitted native entity.
2. Starts a new trace automatically.
3. Seeds that new trace with the previous fitted entity endpoint.
4. Adds the newly captured point as the second point in the new trace.

This makes workflows like this work correctly:

```text
Capture arc start
Capture arc end
Center Arc
Jog to tangent line endpoint
Capture Point
```

The native DXF ARC remains a real arc, and the new tangent segment becomes a separate trace/polyline starting from the arc endpoint.

## Notes

- Editing an existing fitted trace with Replace, Insert After, or Delete still clears that trace's native entity and returns it to point/polyline behavior. That is intentional.
- Continuing from a native entity uses the last defining point as the continuation start.
- Motion behavior remains unchanged from v0.4.1/v0.4.0: guarded X/Y-only controlled moves.

## Install drop-in files

```bash
cd ~/projects/FabScan
cp /path/to/unzipped/FabScan_v043_continue_native_dropin/fabscan/app.py fabscan/app.py
cp /path/to/unzipped/FabScan_v043_continue_native_dropin/README_v0.4.3.md README.md
```

Then test:

```bash
source .venv/bin/activate
python3 -m py_compile fabscan/*.py fabscan.py
python3 fabscan.py
```

## Commit

```bash
git status
git add fabscan/app.py README.md
git commit -m "Preserve native trace geometry when continuing traces"
git push
```
