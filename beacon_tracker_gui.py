"""
Live NRF24 beacon tracker GUI.

Install dependencies:
    python -m pip install pyserial matplotlib

Run:
    python python/beacon_tracker_gui.py --port COM7
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


ANCHORS = [(0.0, 28.87), (-25.0, -14.43), (25.0, -14.43)]


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: queue.Queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = threading.Event()

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                time.sleep(2.0)
                ser.reset_input_buffer()
                self.out_queue.put(("status", f"Connected to {self.port}"))
                while not self.stop_event.is_set():
                    raw = ser.readline().decode("utf-8", errors="replace").strip()
                    if not raw:
                        continue
                    if raw.startswith("{"):
                        try:
                            self.out_queue.put(("fix", json.loads(raw)))
                        except json.JSONDecodeError:
                            self.out_queue.put(("status", f"Bad JSON: {raw[:80]}"))
                    elif raw.startswith("RAW"):
                        self.out_queue.put(("raw", raw))
                    else:
                        self.out_queue.put(("status", raw))
        except serial.SerialException as exc:
            self.out_queue.put(("status", f"Serial error: {exc}"))

    def stop(self) -> None:
        self.stop_event.set()


class TrackerApp:
    def __init__(self, root: tk.Tk, port: str, baud: int) -> None:
        self.root = root
        self.root.title("NRF24 Beacon Tracker")
        self.events: queue.Queue = queue.Queue()
        self.reader = SerialReader(port, baud, self.events)
        self.last_fix: dict | None = None
        self.path_x: list[float] = []
        self.path_y: list[float] = []

        self.status_var = tk.StringVar(value="Opening serial port...")
        self.fix_var = tk.StringVar(value="No fix yet")
        self.dist_var = tk.StringVar(value="Distances: --")
        self.raw_var = tk.StringVar(value="RAW: --")

        self._build_ui()
        self.reader.start()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.process_events)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(7.2, 6.2), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-90, 90)
        self.ax.set_ylim(-80, 90)
        self.ax.grid(True, color="#d0d0d0", linewidth=0.8)
        self.ax.set_xlabel("x cm")
        self.ax.set_ylabel("y cm")

        self.canvas = FigureCanvasTkAgg(self.fig, master=main)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        side = ttk.Frame(main, padding=(12, 0, 0, 0))
        side.pack(side=tk.RIGHT, fill=tk.Y)
        ttk.Label(side, textvariable=self.status_var, width=36).pack(anchor="w", pady=(0, 10))
        ttk.Label(side, textvariable=self.fix_var, width=36).pack(anchor="w", pady=(0, 10))
        ttk.Label(side, textvariable=self.dist_var, width=36).pack(anchor="w", pady=(0, 10))
        ttk.Label(side, textvariable=self.raw_var, width=36).pack(anchor="w", pady=(0, 10))
        ttk.Button(side, text="Clear path", command=self.clear_path).pack(anchor="w", pady=(8, 0))

        self.draw_plot()

    def clear_path(self) -> None:
        self.path_x.clear()
        self.path_y.clear()
        self.draw_plot()

    def process_events(self) -> None:
        changed = False
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if kind == "fix":
                self.last_fix = payload
                self.update_labels(payload)
                if payload.get("fix"):
                    self.path_x.append(float(payload.get("sx", payload.get("x", 0.0))))
                    self.path_y.append(float(payload.get("sy", payload.get("y", 0.0))))
                    self.path_x = self.path_x[-200:]
                    self.path_y = self.path_y[-200:]
                changed = True
            elif kind == "raw":
                self.raw_var.set(payload[:70])
            else:
                self.status_var.set(str(payload)[:90])

        if changed:
            self.draw_plot()
        self.root.after(50, self.process_events)

    def update_labels(self, fix: dict) -> None:
        if fix.get("fix"):
            self.fix_var.set(
                f"Fix: x={fix.get('sx', 0):.1f} cm, y={fix.get('sy', 0):.1f} cm"
            )
        else:
            self.fix_var.set("Fix: waiting for all 3 anchors")

        d = fix.get("d", [math.nan, math.nan, math.nan])
        self.dist_var.set(f"Distances: A0={d[0]:.1f}, A1={d[1]:.1f}, A2={d[2]:.1f} cm")
        self.status_var.set(
            f"Packets={fix.get('pkts', 0)}  solves={fix.get('solves', 0)}  "
            f"valid={fix.get('fix_pct', 0):.1f}%"
        )

    def draw_plot(self) -> None:
        self.ax.clear()
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-90, 90)
        self.ax.set_ylim(-80, 90)
        self.ax.grid(True, color="#d0d0d0", linewidth=0.8)
        self.ax.set_xlabel("x cm")
        self.ax.set_ylabel("y cm")
        self.ax.set_title("Beacon position, triangle side = 50 cm")

        ax_x, ax_y = zip(*ANCHORS)
        self.ax.plot([*ax_x, ax_x[0]], [*ax_y, ax_y[0]], color="#555555", linewidth=1.2)
        self.ax.scatter(ax_x, ax_y, s=90, color="#222222", label="Anchors", zorder=4)
        for i, (x, y) in enumerate(ANCHORS):
            self.ax.text(x + 2, y + 2, f"A{i}", fontsize=10)

        if self.last_fix:
            dists = self.last_fix.get("d", [])
            for (x, y), dist in zip(ANCHORS, dists):
                if dist < 490:
                    circle = self.ax.add_patch(
                        plt_circle((x, y), dist, color="#2f80ed", alpha=0.10)
                    )
                    circle.set_linewidth(1.0)

        if self.path_x and self.path_y:
            self.ax.plot(self.path_x, self.path_y, color="#f2994a", linewidth=1.5, alpha=0.8)
            self.ax.scatter(
                [self.path_x[-1]], [self.path_y[-1]],
                s=130, color="#eb5757", edgecolor="white", linewidth=1.2,
                label="Beacon", zorder=5
            )

        self.ax.legend(loc="upper right")
        self.canvas.draw_idle()

    def close(self) -> None:
        self.reader.stop()
        self.root.destroy()


def plt_circle(center: tuple[float, float], radius: float, color: str, alpha: float):
    from matplotlib.patches import Circle

    return Circle(center, radius, fill=False, color=color, alpha=alpha)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NRF24 beacon tracker GUI")
    parser.add_argument("--port", required=True, help="ESP32 serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    TrackerApp(root, args.port, args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()
