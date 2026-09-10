## Camera Hardware and Compatibility

FabScan does not require a specific camera model, but development and physical-machine testing have been performed with a USB digital microscope originally purchased in 2017.

The original Amazon product is no longer sold, but the archived listing is:

**Amazon ASIN:** `B01N4AUFSC`

The camera identifies itself generically to Linux as:

```
USB Camera: USB Camera
Driver: uvcvideo
Interface: V4L2 / UVC
```

This is useful because FabScan does not depend on a proprietary camera driver. A current replacement should preferably be a standard USB Video Class (UVC) camera that Linux exposes through V4L2.

### Known-Good Development Mode

The current FabScan development configuration uses:

```
Resolution:     800 × 600
Pixel format:   MJPG
Frame rate:     30 FPS
Driver:         Linux uvcvideo
Camera API:     V4L2
Focus:          Manual
```

This configuration has been used for the current FabScan v0.6 development, simulator calibration, record/replay datasets, and physical tracing tests.

### Supported Modes of the Development Camera

The camera reports the following useful modes.

**YUYV / uncompressed**

```
640 × 480       30 FPS
800 × 600       20 FPS
1280 × 960       9 FPS
1600 × 1200      5 FPS
```

**MJPG / compressed**

```
640 × 480       up to 30 FPS
800 × 600       up to 30 FPS
1280 × 960      up to 30 FPS
1600 × 1200     up to 30 FPS
```

The camera does **not** provide a 1280 × 720 mode.

This is worth noting because requesting an unsupported camera resolution can cause V4L2/OpenCV capture failures or sluggish behavior.

FabScan therefore uses 800 × 600 MJPG as the current known-good baseline rather than simply requesting the camera's highest advertised resolution.

### Camera Controls Reported by Linux

The development camera exposes standard V4L2 controls including:

* Brightness
* Contrast
* Saturation
* Hue
* Automatic white balance
* Gamma
* Gain
* Power-line frequency
* White-balance temperature
* Sharpness
* Backlight compensation

Focus is adjusted mechanically on the microscope rather than through V4L2.

### What Matters When Choosing Another Camera

A replacement camera does not need to match the original model exactly.

The most important characteristics for FabScan are:

1. **Standard UVC / V4L2 support**

   The camera should appear as a normal Linux video device using the `uvcvideo` driver whenever possible.

2. **Stable video modes**

   A reliable 640 × 480 or 800 × 600 stream is more useful than a high advertised maximum resolution that cannot maintain a stable frame rate.

3. **Manual focus**

   Manual focus is desirable because the camera-to-work distance is essentially fixed during tracing. Automatic focus hunting during a trace could change the apparent profile position or image quality.

4. **Rigid mounting**

   Mechanical rigidity is at least as important as camera resolution. Any movement between the camera and machine coordinates directly affects tracing accuracy.

5. **Adequate working distance**

   The camera must be capable of focusing at a practical mounting height while providing enough field of view for FabScan's look-ahead planner.

6. **Controllable lighting**

   Built-in adjustable LEDs are useful, although even illumination and avoiding glare/shadows are more important than maximum brightness.

7. **MJPG support is desirable**

   On the known-good camera, MJPG allows 800 × 600 at 30 FPS while YUYV is limited to 20 FPS at the same resolution.

### Evaluating an Unknown Camera

On Linux, a prospective camera can be checked with:

```
v4l2-ctl --all
```

and:

```
v4l2-ctl --list-formats-ext
```

Useful signs are:

```
Driver: uvcvideo
Video Capture
Streaming
MJPG support
640×480 or 800×600 at 20–30 FPS
```

Do not assume that a resolution listed in advertising is actually available through V4L2.

### Possible Current Replacement

A current 2 MP-class USB microscope such as the Teslong USB microscope family appears to have the basic characteristics FabScan needs: manual focus, USB/UVC-style video, adjustable lighting, and sufficient resolution.

However, no current replacement model should be considered **FabScan validated** until it has been tested on Linux and its actual V4L2 modes have been confirmed.

The original 2017 camera remains the current known-good reference hardware.
