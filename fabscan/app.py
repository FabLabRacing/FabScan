from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from fabscan.dxf_export import export_contours_to_dxf
from fabscan.image_processing import FoundContour, ProcessedImage, find_contours
from fabscan.scale_tools import ScaleResult, calculate_scale


ImagePoint = Tuple[float, float]


class FabScanApp(tk.Tk):
    """FabScan Ver. 0.1 desktop app.

    This intentionally favors simple and debuggable over pretty. The goal is to
    prove the photo/scan -> contours -> scaled DXF workflow.
    """

    def __init__(self) -> None:
        super().__init__()
        self.title("FabScan Ver. 0.1 - First Swing")
        self.geometry("1200x820")
        self.minsize(1000, 700)

        self.image_path: Optional[Path] = None
        self.image_bgr: Optional[np.ndarray] = None
        self.processed: Optional[ProcessedImage] = None
        self.scale_result: Optional[ScaleResult] = None
        self.scale_points: list[ImagePoint] = []
        self.scale_mode = False

        self.display_scale = 1.0
        self.display_offset_x = 0.0
        self.display_offset_y = 0.0
        self._tk_image: Optional[ImageTk.PhotoImage] = None

        self.threshold_var = tk.IntVar(value=127)
        self.blur_var = tk.IntVar(value=3)
        self.min_area_var = tk.DoubleVar(value=1000.0)
        self.simplify_var = tk.DoubleVar(value=0.20)
        self.invert_var = tk.BooleanVar(value=False)
        self.show_threshold_var = tk.BooleanVar(value=False)

        self._build_ui()

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=8)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        ttk.Button(toolbar, text="Load Image", command=self.load_image).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(toolbar, text="Find Contours", command=self.process_image).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Set Scale", command=self.start_scale_mode).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Export DXF", command=self.export_dxf).pack(side=tk.LEFT, padx=6)

        ttk.Checkbutton(
            toolbar,
            text="Invert",
            variable=self.invert_var,
            command=self.process_image_if_loaded,
        ).pack(side=tk.LEFT, padx=(18, 6))

        ttk.Checkbutton(
            toolbar,
            text="Show Threshold",
            variable=self.show_threshold_var,
            command=self.redraw_preview,
        ).pack(side=tk.LEFT, padx=6)

        controls = ttk.LabelFrame(self, text="Image Cleanup", padding=8)
        controls.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 8))

        self._add_slider(controls, "Threshold", self.threshold_var, 0, 255, 1, 0)
        self._add_slider(controls, "Blur", self.blur_var, 1, 21, 1, 1)
        self._add_slider(controls, "Min Area", self.min_area_var, 0, 50000, 100, 2)
        self._add_slider(controls, "Simplify %", self.simplify_var, 0.01, 5.0, 0.01, 3)

        main = ttk.Frame(self, padding=(8, 0, 8, 8))
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main, bg="#202020", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Configure>", lambda _event: self.redraw_preview())

        side = ttk.Frame(main, padding=(8, 0, 0, 0), width=240)
        side.pack(side=tk.RIGHT, fill=tk.Y)
        side.pack_propagate(False)

        ttk.Label(side, text="Status", font=("TkDefaultFont", 11, "bold")).pack(anchor=tk.W)
        self.status_text = tk.Text(side, height=20, width=34, wrap=tk.WORD)
        self.status_text.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        self.status_text.configure(state=tk.DISABLED)

        self.set_status(
            "Load a clean photo/scan of a flat part.\n\n"
            "Tip: For Ver. 0.1, a high-contrast image with the part separated "
            "from the background will work best."
        )

    def _add_slider(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.Variable,
        from_value: float,
        to_value: float,
        resolution: float,
        column: int,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="ew", padx=6)
        parent.columnconfigure(column, weight=1)

        ttk.Label(frame, text=label).pack(anchor=tk.W)
        scale = ttk.Scale(
            frame,
            from_=from_value,
            to=to_value,
            variable=variable,
            command=lambda _value: self.redraw_preview(),
        )
        scale.pack(fill=tk.X)

        entry = ttk.Entry(frame, textvariable=variable, width=10)
        entry.pack(anchor=tk.W, pady=(2, 0))

    def set_status(self, text: str) -> None:
        self.status_text.configure(state=tk.NORMAL)
        self.status_text.delete("1.0", tk.END)
        self.status_text.insert(tk.END, text)
        self.status_text.configure(state=tk.DISABLED)

    def append_status(self, text: str) -> None:
        self.status_text.configure(state=tk.NORMAL)
        self.status_text.insert(tk.END, "\n" + text)
        self.status_text.configure(state=tk.DISABLED)
        self.status_text.see(tk.END)

    def load_image(self) -> None:
        path = filedialog.askopenfilename(
            title="Load part image",
            filetypes=(
                ("Image files", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff"),
                ("All files", "*.*"),
            ),
        )
        if not path:
            return

        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            messagebox.showerror("Load failed", "OpenCV could not load that image.")
            return

        self.image_path = Path(path)
        self.image_bgr = image
        self.processed = None
        self.scale_result = None
        self.scale_points = []
        self.scale_mode = False

        h, w = image.shape[:2]
        self.set_status(f"Loaded:\n{self.image_path.name}\n\nImage size: {w} x {h} px")
        self.redraw_preview()

    def process_image_if_loaded(self) -> None:
        if self.image_bgr is not None:
            self.process_image()

    def process_image(self) -> None:
        if self.image_bgr is None:
            messagebox.showinfo("No image", "Load an image first.")
            return

        try:
            self.processed = find_contours(
                image_bgr=self.image_bgr,
                threshold_value=int(self.threshold_var.get()),
                blur_size=int(self.blur_var.get()),
                invert=bool(self.invert_var.get()),
                min_area=float(self.min_area_var.get()),
                simplify_percent=float(self.simplify_var.get()),
            )
        except Exception as exc:  # noqa: BLE001 - show the real problem in the UI
            messagebox.showerror("Processing failed", str(exc))
            return

        outside = sum(1 for c in self.processed.contours if c.layer == "OUTSIDE")
        inside = sum(1 for c in self.processed.contours if c.layer == "INSIDE")
        total_points = sum(len(c.points) for c in self.processed.contours)

        scale_text = "Not set"
        if self.scale_result is not None:
            scale_text = f"{self.scale_result.inches_per_pixel:.8f} in/px"

        self.set_status(
            f"Contours found: {len(self.processed.contours)}\n"
            f"Outside: {outside}\n"
            f"Inside: {inside}\n"
            f"Total points: {total_points}\n\n"
            f"Threshold: {int(self.threshold_var.get())}\n"
            f"Blur: {int(self.blur_var.get())}\n"
            f"Min area: {float(self.min_area_var.get()):.1f}\n"
            f"Simplify: {float(self.simplify_var.get()):.2f}%\n\n"
            f"Scale: {scale_text}"
        )
        self.redraw_preview()

    def start_scale_mode(self) -> None:
        if self.image_bgr is None:
            messagebox.showinfo("No image", "Load an image first.")
            return
        self.scale_mode = True
        self.scale_points = []
        self.scale_result = None
        self.append_status("\nScale mode: click two known points on the image.")
        self.redraw_preview()

    def on_canvas_click(self, event: tk.Event) -> None:
        if not self.scale_mode or self.image_bgr is None:
            return

        image_point = self.canvas_to_image_point(event.x, event.y)
        if image_point is None:
            return

        self.scale_points.append(image_point)
        self.redraw_preview()

        if len(self.scale_points) == 2:
            known = simpledialog.askfloat(
                "Known distance",
                "Enter the real distance between the two points in inches:",
                minvalue=0.0001,
            )
            if known is None:
                self.scale_points = []
                self.scale_mode = False
                self.redraw_preview()
                return

            try:
                self.scale_result = calculate_scale(self.scale_points[0], self.scale_points[1], known)
            except Exception as exc:  # noqa: BLE001
                messagebox.showerror("Scale failed", str(exc))
                self.scale_points = []
                self.scale_mode = False
                self.redraw_preview()
                return

            self.scale_mode = False
            self.append_status(
                f"\nScale set:\n"
                f"Pixels: {self.scale_result.pixels:.3f}\n"
                f"Known distance: {self.scale_result.inches:.4f} in\n"
                f"Scale: {self.scale_result.inches_per_pixel:.8f} in/px"
            )
            self.redraw_preview()

    def canvas_to_image_point(self, canvas_x: float, canvas_y: float) -> Optional[ImagePoint]:
        if self.image_bgr is None:
            return None

        h, w = self.image_bgr.shape[:2]
        x = (canvas_x - self.display_offset_x) / self.display_scale
        y = (canvas_y - self.display_offset_y) / self.display_scale

        if x < 0 or y < 0 or x >= w or y >= h:
            return None
        return (x, y)

    def image_to_canvas_point(self, image_x: float, image_y: float) -> Tuple[float, float]:
        return (
            self.display_offset_x + image_x * self.display_scale,
            self.display_offset_y + image_y * self.display_scale,
        )

    def export_dxf(self) -> None:
        if self.image_bgr is None:
            messagebox.showinfo("No image", "Load an image first.")
            return

        if self.processed is None or not self.processed.contours:
            self.process_image()
            if self.processed is None or not self.processed.contours:
                messagebox.showinfo("No contours", "No contours found to export.")
                return

        if self.scale_result is None:
            messagebox.showinfo("Scale required", "Set the scale before exporting a DXF.")
            return

        default_name = "fabscan_export.dxf"
        if self.image_path is not None:
            default_name = self.image_path.stem + ".dxf"

        path = filedialog.asksaveasfilename(
            title="Export DXF",
            defaultextension=".dxf",
            initialfile=default_name,
            initialdir=str(Path.cwd() / "exports"),
            filetypes=(("DXF files", "*.dxf"), ("All files", "*.*")),
        )
        if not path:
            return

        try:
            image_height = int(self.image_bgr.shape[0])
            output = export_contours_to_dxf(
                contours=self.processed.contours,
                output_path=path,
                scale_inches_per_pixel=self.scale_result.inches_per_pixel,
                image_height_pixels=image_height,
            )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("DXF export failed", str(exc))
            return

        self.append_status(f"\nDXF exported:\n{output}")
        messagebox.showinfo("DXF exported", f"Saved:\n{output}")

    def redraw_preview(self) -> None:
        self.canvas.delete("all")
        if self.image_bgr is None:
            self.canvas.create_text(
                self.canvas.winfo_width() / 2,
                self.canvas.winfo_height() / 2,
                text="Load an image to start",
                fill="white",
                font=("TkDefaultFont", 18),
            )
            return

        display_rgb = self._make_display_rgb()
        pil_image = Image.fromarray(display_rgb)

        canvas_w = max(1, self.canvas.winfo_width())
        canvas_h = max(1, self.canvas.winfo_height())
        img_w, img_h = pil_image.size
        self.display_scale = min(canvas_w / img_w, canvas_h / img_h)
        new_w = max(1, int(img_w * self.display_scale))
        new_h = max(1, int(img_h * self.display_scale))
        self.display_offset_x = (canvas_w - new_w) / 2
        self.display_offset_y = (canvas_h - new_h) / 2

        pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)
        draw = ImageDraw.Draw(pil_image)
        self._draw_overlay(draw)

        self._tk_image = ImageTk.PhotoImage(pil_image)
        self.canvas.create_image(
            self.display_offset_x,
            self.display_offset_y,
            anchor=tk.NW,
            image=self._tk_image,
        )

    def _make_display_rgb(self) -> np.ndarray:
        if self.image_bgr is None:
            raise RuntimeError("No image loaded")

        if self.show_threshold_var.get() and self.processed is not None:
            threshold = self.processed.threshold_image
            return cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB)

        return cv2.cvtColor(self.image_bgr, cv2.COLOR_BGR2RGB)

    def _draw_overlay(self, draw: ImageDraw.ImageDraw) -> None:
        if self.processed is not None:
            for contour in self.processed.contours:
                self._draw_contour(draw, contour)

        if self.scale_points:
            scaled_points = [
                (x * self.display_scale, y * self.display_scale) for x, y in self.scale_points
            ]
            for x, y in scaled_points:
                radius = 5
                draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline="yellow", width=2)
            if len(scaled_points) == 2:
                draw.line(scaled_points, fill="yellow", width=2)

    def _draw_contour(self, draw: ImageDraw.ImageDraw, contour: FoundContour) -> None:
        points = [(x * self.display_scale, y * self.display_scale) for x, y in contour.points]
        if len(points) < 2:
            return
        color = "lime" if contour.layer == "OUTSIDE" else "cyan"
        draw.line(points + [points[0]], fill=color, width=2)


def main() -> None:
    app = FabScanApp()
    app.mainloop()


if __name__ == "__main__":
    main()
