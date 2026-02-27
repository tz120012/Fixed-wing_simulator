"""
gui_tuner.py  –  Interactive PID parameter tuning GUI (Tkinter).

Layout
------
┌──────────────────────────────────────────────────────────────────────┐
│  Fixed-Wing PID Tuner                          [Save] [Load] [Reset] │
├────────────────────┬──────────────────────────────────────────────────┤
│  PARAMETER PANEL   │   REAL-TIME RESPONSE CURVES                     │
│                    │                                                  │
│  ┌─ Pitch Attitude ┐│   ┌─ Step Response ────────────────────────┐   │
│  │ PTCH_P  [1.00]  ││   │  output ── setpoint                    │   │
│  │ PTCH_D  [0.08]  ││   └────────────────────────────────────────┘   │
│  └─────────────────┘│                                                  │
│  ┌─ Pitch Rate ─────┐│   ┌─ Error ────────────────────────────────┐  │
│  │ PTCH_RATE_P[0.04]││   │                                        │  │
│  │ PTCH_RATE_I[0.10]││   └────────────────────────────────────────┘  │
│  │ PTCH_RATE_D[0.00]││                                                │
│  └─────────────────┘│   ┌─ PID Terms ────────────────────────────┐  │
│  … (other groups)  │   │  P ── I ── D                           │  │
│                    │   └────────────────────────────────────────┘   │
│  [Axis: Pitch ▼]   │                                                  │
│  [▶ Run Sim] [■ Stop]│  Overshoot: 3.2%   Settle: 1.45 s            │
│  [🤖 RL Auto-tune]  │  IAE: 0.234                                    │
└────────────────────┴──────────────────────────────────────────────────┘

Usage
-----
python -m pid_tuner.gui_tuner               # standalone
python -m pid_tuner.gui_tuner --axis pitch  # start on pitch tab
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np

# Matplotlib embedding
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

# Local imports
_HERE = os.path.dirname(os.path.abspath(__file__))          # .../extended_application/pid_tuner
_EXT  = os.path.dirname(_HERE)                              # .../extended_application
_ROOT = os.path.dirname(_EXT)                               # .../FixedWingSimulator
sys.path.insert(0, os.path.join(_ROOT, "src"))              # FixedWingSimulator/src
sys.path.insert(0, _EXT)                                    # extended_application (for pid_tuner pkg)

from pid_tuner.param_store import ParamStore, PARAM_GROUPS, PARAM_RANGES, DEFAULT_PARAMS
from pid_tuner.rl_env      import PIDTuningEnv, _GAIN_RANGES


# ---------------------------------------------------------------------------
# Colour scheme
# ---------------------------------------------------------------------------
COLORS = {
    "bg":         "#1e1e2e",
    "panel":      "#2a2a3e",
    "accent":     "#89b4fa",
    "green":      "#a6e3a1",
    "red":        "#f38ba8",
    "yellow":     "#f9e2af",
    "text":       "#cdd6f4",
    "muted":      "#6c7086",
    "entry_bg":   "#313244",
    "entry_fg":   "#cdd6f4",
}


# ---------------------------------------------------------------------------
# Simulation worker (runs in background thread)
# ---------------------------------------------------------------------------

class _SimWorker:
    """
    Runs the PIDTuningEnv step-by-step in a background thread,
    continuously posting (t, ref, output, error, p_term, i_term, d_term)
    to a ring buffer that the GUI reads.
    """

    def __init__(self, store: ParamStore, axis: str = "pitch"):
        self.store  = store
        self.axis   = axis
        self._env   = PIDTuningEnv(axis=axis, episode_steps=99999, dt=0.01)
        self._lock  = threading.Lock()
        self._buf: List[Dict] = []
        self._max_buf = 600   # 6 seconds at dt=0.01
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._metrics: Dict[str, float] = {}
        self._version = -1    # track ParamStore changes
        # History: keep last N snapshots for comparison overlay
        self._history: List[List[Dict]] = []   # each entry is a full buffer snapshot
        self._max_history = 3                  # keep at most 3 previous curves

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._env.reset()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def restart(self) -> None:
        self.stop()
        time.sleep(0.05)
        self._env.reset()
        with self._lock:
            self._buf.clear()
            self._history.clear()
        self._metrics = {}
        self._version = -1
        self.start()

    def change_axis(self, axis: str) -> None:
        self.stop()
        time.sleep(0.05)
        self.axis  = axis
        self._env  = PIDTuningEnv(axis=axis, episode_steps=99999, dt=0.01)
        self.restart()

    def get_buffer(self) -> List[Dict]:
        with self._lock:
            return list(self._buf)

    def get_history(self) -> List[List[Dict]]:
        """Return snapshots of previous response curves (for overlay)."""
        with self._lock:
            return [list(snap) for snap in self._history]

    def get_metrics(self) -> Dict[str, float]:
        return dict(self._metrics)

    # ------------------------------------------------------------------

    def _run(self) -> None:
        obs, _ = self._env.reset()
        action = np.zeros(3)  # no RL action – fixed gains

        while self._running:
            # Check for parameter update
            if self.store.version != self._version:
                self._version = self.store.version
                gains = self.store.get_all()
                gr = _GAIN_RANGES[self.axis]
                # Map ArduPilot names to env PID
                axis_map = {
                    "pitch": ("PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D"),
                    "roll":  ("ROLL_RATE_P",  "ROLL_RATE_I",  "ROLL_D"),
                    "yaw":   ("YAW_RATE_P",   "YAW_RATE_I",   "YAW_P"),
                }
                kp_key, ki_key, kd_key = axis_map.get(self.axis, axis_map["pitch"])
                self._env._kp = np.clip(gains.get(kp_key, self._env._kp), *gr["kp"])
                self._env._ki = np.clip(gains.get(ki_key, self._env._ki), *gr["ki"])
                self._env._kd = np.clip(gains.get(kd_key, self._env._kd), *gr["kd"])
                self._env._pid.kp = self._env._kp
                self._env._pid.ki = self._env._ki
                self._env._pid.kd = self._env._kd
                # Save current buffer as a history snapshot, then reset env so
                # that _t restarts from 0 and stays within the fixed X-axis [0, XWINDOW]
                with self._lock:
                    if len(self._buf) >= 10:   # only save if there's meaningful data
                        self._history.append(list(self._buf))
                        if len(self._history) > self._max_history:
                            self._history.pop(0)
                    self._buf.clear()
                obs, _ = self._env.reset()   # resets _t → 0 and clears PID integrator

            obs, reward, term, trunc, info = self._env.step(action)

            p_term = self._env._pid.kp * info["error"]
            i_term = float(self._env._pid._integral)
            d_term = float(self._env._pid.kd * (self._env._pid._prev_err - info["error"])
                           / max(self._env._pid.dt, 1e-9))

            record = {
                "t":       self._env._t,
                "ref":     info["ref"],
                "output":  info["output"],
                "error":   info["error"],
                "p_term":  p_term,
                "i_term":  i_term,
                "d_term":  d_term,
            }
            with self._lock:
                self._buf.append(record)
                if len(self._buf) > self._max_buf:
                    self._buf.pop(0)

            self._metrics = {
                "overshoot":  info["peak_overshoot"] * 100.0,
                "settle_time": info["settle_time"],
                "iae":        info["integral_abs_error"],
            }

            if term or trunc:
                obs, _ = self._env.reset()

            time.sleep(self._env._pid.dt)


# ---------------------------------------------------------------------------
# Parameter entry widget (label + slider + entry, all synced)
# ---------------------------------------------------------------------------

class _ParamRow:
    def __init__(self, parent: tk.Frame, name: str, store: ParamStore,
                 row: int, on_change):
        self._name  = name
        self._store = store
        self._on_change = on_change

        lo, hi = PARAM_RANGES.get(name, (0.0, 10.0))
        val = store.get(name)

        # Label
        lbl = tk.Label(parent, text=name, anchor="w", width=18,
                       bg=COLORS["panel"], fg=COLORS["text"],
                       font=("Consolas", 9))
        lbl.grid(row=row, column=0, padx=(8,4), pady=2, sticky="w")

        # Slider
        self._var = tk.DoubleVar(value=val)
        self._var.trace_add("write", self._on_slider)
        slider = ttk.Scale(parent, from_=lo, to=hi, variable=self._var,
                           orient="horizontal", length=140)
        slider.grid(row=row, column=1, padx=4, pady=2)

        # Entry
        self._entry_var = tk.StringVar(value=f"{val:.4f}")
        entry = tk.Entry(parent, textvariable=self._entry_var, width=8,
                         bg=COLORS["entry_bg"], fg=COLORS["entry_fg"],
                         insertbackground=COLORS["text"],
                         relief="flat", font=("Consolas", 9))
        entry.grid(row=row, column=2, padx=(4,8), pady=2)
        entry.bind("<Return>",    self._on_entry)
        entry.bind("<FocusOut>",  self._on_entry)

        self._updating = False

    def _on_slider(self, *_):
        if self._updating:
            return
        val = round(self._var.get(), 5)
        self._updating = True
        self._entry_var.set(f"{val:.4f}")
        self._updating = False
        self._store.set(self._name, val)
        self._on_change()

    def _on_entry(self, *_):
        if self._updating:
            return
        try:
            val = float(self._entry_var.get())
        except ValueError:
            return
        lo, hi = PARAM_RANGES.get(self._name, (0.0, 10.0))
        val = max(lo, min(hi, val))
        self._updating = True
        self._var.set(val)
        self._entry_var.set(f"{val:.4f}")
        self._updating = False
        self._store.set(self._name, val)
        self._on_change()

    def refresh(self) -> None:
        """Refresh display from store (called after load / reset / RL update)."""
        val = self._store.get(self._name)
        self._updating = True
        self._var.set(val)
        self._entry_var.set(f"{val:.4f}")
        self._updating = False


# ---------------------------------------------------------------------------
# Main GUI class
# ---------------------------------------------------------------------------

class PIDTunerGUI:
    """
    Main Tkinter window.

    Parameters
    ----------
    store   : shared ParamStore (pass the same instance to your simulator)
    axis    : initial axis ("pitch" | "roll" | "yaw")
    """

    def __init__(self, store: Optional[ParamStore] = None, axis: str = "pitch"):
        self.store  = store or ParamStore()
        self._axis  = axis

        # Simulation worker
        self._worker = _SimWorker(self.store, axis=axis)

        # Build window
        self.root = tk.Tk()
        self.root.title("Fixed-Wing PID Tuner")
        self.root.configure(bg=COLORS["bg"])
        self.root.resizable(True, True)
        self.root.geometry("1180x680")

        self._setup_styles()
        self._build_toolbar()
        self._build_main()

        # Param rows registry
        self._rows: Dict[str, _ParamRow] = {}
        self._build_param_panel()
        self._build_chart_panel()

        # Start periodic chart update (every 150 ms)
        self.root.after(150, self._update_charts)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # Styles
    # ------------------------------------------------------------------

    def _setup_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TScale",
                        background=COLORS["panel"],
                        troughcolor=COLORS["bg"],
                        slidercolor=COLORS["accent"])
        style.configure("Toolbar.TButton",
                        background=COLORS["panel"],
                        foreground=COLORS["text"],
                        padding=4)
        style.configure("Accent.TButton",
                        background=COLORS["accent"],
                        foreground=COLORS["bg"],
                        padding=4, font=("Helvetica", 9, "bold"))
        style.configure("Green.TButton",
                        background=COLORS["green"],
                        foreground=COLORS["bg"],
                        padding=4)
        style.configure("Red.TButton",
                        background=COLORS["red"],
                        foreground=COLORS["bg"],
                        padding=4)

    # ------------------------------------------------------------------
    # Toolbar
    # ------------------------------------------------------------------

    def _build_toolbar(self) -> None:
        bar = tk.Frame(self.root, bg=COLORS["panel"], height=36)
        bar.pack(fill="x", side="top", pady=(0,1))

        tk.Label(bar, text="⚙  Fixed-Wing PID Tuner",
                 bg=COLORS["panel"], fg=COLORS["accent"],
                 font=("Helvetica", 12, "bold")).pack(side="left", padx=12)

        # Right buttons
        for txt, cmd, style in [
            ("💾 Save", self._save_params, "Toolbar.TButton"),
            ("📂 Load", self._load_params, "Toolbar.TButton"),
            ("↺ Reset", self._reset_params, "Toolbar.TButton"),
        ]:
            ttk.Button(bar, text=txt, command=cmd, style=style).pack(
                side="right", padx=4, pady=4)

    # ------------------------------------------------------------------
    # Main paned layout
    # ------------------------------------------------------------------

    def _build_main(self) -> None:
        paned = ttk.PanedWindow(self.root, orient="horizontal")
        paned.pack(fill="both", expand=True, padx=8, pady=6)

        # Left panel frame
        self._left = tk.Frame(paned, bg=COLORS["panel"], width=310)
        self._left.pack_propagate(False)
        paned.add(self._left, weight=0)

        # Right frame
        self._right = tk.Frame(paned, bg=COLORS["bg"])
        paned.add(self._right, weight=1)

    # ------------------------------------------------------------------
    # Parameter panel (left)
    # ------------------------------------------------------------------

    def _build_param_panel(self) -> None:
        left = self._left

        # Axis selector
        axis_frame = tk.Frame(left, bg=COLORS["panel"])
        axis_frame.pack(fill="x", padx=8, pady=(8,4))
        tk.Label(axis_frame, text="Axis:", bg=COLORS["panel"],
                 fg=COLORS["text"]).pack(side="left")
        self._axis_var = tk.StringVar(value=self._axis.capitalize())
        axis_cb = ttk.Combobox(axis_frame, textvariable=self._axis_var,
                               values=["Pitch", "Roll", "Yaw"], width=8,
                               state="readonly")
        axis_cb.pack(side="left", padx=6)
        axis_cb.bind("<<ComboboxSelected>>", self._on_axis_change)

        # Scrollable param area
        canvas_frame = tk.Frame(left, bg=COLORS["panel"])
        canvas_frame.pack(fill="both", expand=True)
        canvas = tk.Canvas(canvas_frame, bg=COLORS["panel"],
                           highlightthickness=0)
        scroll = ttk.Scrollbar(canvas_frame, orient="vertical",
                               command=canvas.yview)
        self._scroll_frame = tk.Frame(canvas, bg=COLORS["panel"])
        self._scroll_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self._scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self._param_canvas = canvas

        self._populate_param_rows()

        # Bottom control buttons
        btn_frame = tk.Frame(left, bg=COLORS["panel"])
        btn_frame.pack(fill="x", padx=8, pady=8)
        ttk.Button(btn_frame, text="▶ Run",  command=self._run_sim,
                   style="Green.TButton").pack(side="left", padx=2)
        ttk.Button(btn_frame, text="■ Stop", command=self._stop_sim,
                   style="Red.TButton").pack(side="left", padx=2)
        ttk.Button(btn_frame, text="🤖 RL Auto-tune",
                   command=self._rl_autotune,
                   style="Accent.TButton").pack(side="left", padx=2)

    def _populate_param_rows(self) -> None:
        """Build parameter rows for current axis."""
        for w in self._scroll_frame.winfo_children():
            w.destroy()
        self._rows.clear()

        axis = self._axis.lower()
        axis_groups = {
            "pitch": ["Pitch Attitude", "Pitch Rate"],
            "roll":  ["Roll Attitude",  "Roll Rate"],
            "yaw":   ["Yaw", "Navigation"],
        }
        groups = axis_groups.get(axis, list(PARAM_GROUPS.keys()))

        row = 0
        for grp_name in groups:
            params = PARAM_GROUPS.get(grp_name, [])
            # Group header
            tk.Label(self._scroll_frame, text=f"── {grp_name}",
                     bg=COLORS["panel"], fg=COLORS["accent"],
                     font=("Helvetica", 9, "bold")).grid(
                row=row, column=0, columnspan=3, padx=8, pady=(8,2), sticky="w")
            row += 1
            for pname in params:
                pr = _ParamRow(self._scroll_frame, pname, self.store,
                               row=row, on_change=lambda: None)
                self._rows[pname] = pr
                row += 1

    def _refresh_all_rows(self) -> None:
        for row in self._rows.values():
            row.refresh()

    # ------------------------------------------------------------------
    # Chart panel (right)
    # ------------------------------------------------------------------

    def _build_chart_panel(self) -> None:
        fig = Figure(figsize=(8, 6), dpi=90,
                     facecolor=COLORS["bg"])
        self._fig = fig

        # Three subplots
        self._ax_resp  = fig.add_subplot(3, 1, 1)
        self._ax_error = fig.add_subplot(3, 1, 2)
        self._ax_pid   = fig.add_subplot(3, 1, 3)
        fig.subplots_adjust(hspace=0.45, left=0.08, right=0.97,
                            top=0.95, bottom=0.08)

        for ax in (self._ax_resp, self._ax_error, self._ax_pid):
            ax.set_facecolor(COLORS["panel"])
            ax.tick_params(colors=COLORS["muted"], labelsize=7)
            ax.spines[:].set_color(COLORS["muted"])
            for spine in ax.spines.values():
                spine.set_linewidth(0.5)

        self._ax_resp.set_title("Step Response", color=COLORS["text"], fontsize=9)
        self._ax_error.set_title("Error", color=COLORS["text"], fontsize=9)
        self._ax_pid.set_title("PID Terms", color=COLORS["text"], fontsize=9)

        canvas = FigureCanvasTkAgg(fig, master=self._right)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas_widget = canvas

        # Metrics label bar
        self._metrics_var = tk.StringVar(value="Overshoot: --   Settle: --   IAE: --")
        tk.Label(self._right, textvariable=self._metrics_var,
                 bg=COLORS["bg"], fg=COLORS["yellow"],
                 font=("Consolas", 9)).pack(side="bottom", pady=4)

    # ------------------------------------------------------------------
    # Periodic chart update (runs in GUI thread via after())
    # ------------------------------------------------------------------

    # Fixed X-axis window (seconds)
    _XWINDOW = 6.0

    def _update_charts(self) -> None:
        buf     = self._worker.get_buffer()
        history = self._worker.get_history()
        metrics = self._worker.get_metrics()

        if len(buf) >= 2:
            t_arr   = np.array([r["t"]      for r in buf])
            ref_arr = np.array([r["ref"]    for r in buf])
            out_arr = np.array([r["output"] for r in buf])
            err_arr = np.array([r["error"]  for r in buf])
            p_arr   = np.array([r["p_term"] for r in buf])
            i_arr   = np.array([r["i_term"] for r in buf])
            d_arr   = np.array([r["d_term"] for r in buf])

            for ax in (self._ax_resp, self._ax_error, self._ax_pid):
                ax.cla()
                ax.set_facecolor(COLORS["panel"])
                ax.tick_params(colors=COLORS["muted"], labelsize=7)
                for spine in ax.spines.values():
                    spine.set_color(COLORS["muted"])
                    spine.set_linewidth(0.5)

            # ── History overlay (faded grey, oldest = most transparent) ──
            n_hist = len(history)
            for i, snap in enumerate(history):
                if len(snap) < 2:
                    continue
                # alpha: oldest 0.15 → newest 0.35
                alpha = 0.15 + 0.20 * (i / max(n_hist - 1, 1))
                ht = np.array([r["t"]      for r in snap])
                ho = np.array([r["output"] for r in snap])
                he = np.array([r["error"]  for r in snap])
                hp = np.array([r["p_term"] for r in snap])
                hi_ = np.array([r["i_term"] for r in snap])
                hd = np.array([r["d_term"] for r in snap])
                self._ax_resp.plot(ht, ho,  color="#888888", lw=0.8, alpha=alpha)
                self._ax_error.plot(ht, he, color="#888888", lw=0.8, alpha=alpha)
                self._ax_pid.plot(ht, hp,   color="#888888", lw=0.6, alpha=alpha)
                self._ax_pid.plot(ht, hi_,  color="#888888", lw=0.6, alpha=alpha)
                self._ax_pid.plot(ht, hd,   color="#888888", lw=0.6, alpha=alpha)

            # ── Current curves ──
            # Response
            self._ax_resp.plot(t_arr, ref_arr, "--",
                               color=COLORS["yellow"], lw=1, label="Setpoint")
            self._ax_resp.plot(t_arr, out_arr,
                               color=COLORS["accent"], lw=1.2, label="Output")
            self._ax_resp.legend(fontsize=7, facecolor=COLORS["panel"],
                                  labelcolor=COLORS["text"], loc="upper right")
            self._ax_resp.set_title("Step Response",
                                     color=COLORS["text"], fontsize=9)

            # Error
            self._ax_error.plot(t_arr, err_arr,
                                 color=COLORS["red"], lw=1)
            self._ax_error.axhline(0, color=COLORS["muted"], lw=0.5, ls="--")
            self._ax_error.set_title("Error", color=COLORS["text"], fontsize=9)

            # PID terms
            self._ax_pid.plot(t_arr, p_arr, color=COLORS["accent"],
                               lw=1, label="P")
            self._ax_pid.plot(t_arr, i_arr, color=COLORS["green"],
                               lw=1, label="I")
            self._ax_pid.plot(t_arr, d_arr, color=COLORS["yellow"],
                               lw=1, label="D")
            self._ax_pid.legend(fontsize=7, facecolor=COLORS["panel"],
                                  labelcolor=COLORS["text"], loc="upper right")
            self._ax_pid.set_title("PID Terms", color=COLORS["text"], fontsize=9)

            # ── Fixed X-axis: always show [0, _XWINDOW] ──
            for ax in (self._ax_resp, self._ax_error, self._ax_pid):
                ax.set_xlim(0.0, self._XWINDOW)

            self._canvas_widget.draw_idle()

        if metrics:
            self._metrics_var.set(
                f"Overshoot: {metrics.get('overshoot', 0):.1f}%   "
                f"Settle: {metrics.get('settle_time', 0):.2f} s   "
                f"IAE: {metrics.get('iae', 0):.3f}"
            )

        self.root.after(150, self._update_charts)

    # ------------------------------------------------------------------
    # Toolbar callbacks
    # ------------------------------------------------------------------

    def _save_params(self) -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml *.yml"), ("JSON", "*.json"), ("All", "*")],
            initialdir=os.path.join(_ROOT, "config"),
            title="Save PID Parameters",
        )
        if not path:
            return
        if path.endswith(".json"):
            self.store.save_json(path)
        else:
            self.store.save_yaml(path)
        messagebox.showinfo("Saved", f"Parameters saved to:\n{path}")

    def _load_params(self) -> None:
        path = filedialog.askopenfilename(
            filetypes=[("YAML", "*.yaml *.yml"), ("JSON", "*.json"), ("All", "*")],
            initialdir=os.path.join(_ROOT, "config"),
            title="Load PID Parameters",
        )
        if not path:
            return
        try:
            if path.endswith(".json"):
                self.store.load_json(path)
            else:
                self.store.load_yaml(path)
            self._refresh_all_rows()
            messagebox.showinfo("Loaded", f"Parameters loaded from:\n{path}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _reset_params(self) -> None:
        if messagebox.askyesno("Reset", "Restore factory defaults?"):
            self.store.reset_defaults()
            self._refresh_all_rows()

    # ------------------------------------------------------------------
    # Simulation control
    # ------------------------------------------------------------------

    def _run_sim(self) -> None:
        self._worker.restart()

    def _stop_sim(self) -> None:
        self._worker.stop()

    def _on_axis_change(self, *_) -> None:
        self._axis = self._axis_var.get().lower()
        self._populate_param_rows()
        self._worker.change_axis(self._axis)

    # ------------------------------------------------------------------
    # RL auto-tune (runs in background thread to avoid blocking GUI)
    # ------------------------------------------------------------------

    def _rl_autotune(self) -> None:
        try:
            from pid_tuner.rl_agent import train
        except ImportError as e:
            messagebox.showerror("RL Error",
                                  f"PyTorch required for RL auto-tuning.\n{e}")
            return

        self._stop_sim()
        messagebox.showinfo(
            "RL Auto-Tune",
            f"Starting PPO training for '{self._axis}' axis.\n"
            "This may take 30–120 seconds depending on your hardware.\n"
            "The GUI will remain responsive. Check console for progress."
        )

        def _run_train():
            ckpt = os.path.join(_ROOT, "pid_tuner", "checkpoints",
                                f"{self._axis}_ppo.pt")
            try:
                agent = train(
                    axis=self._axis,
                    total_steps=20_000,
                    save_path=ckpt,
                    verbose=True,
                )
                # Run one deterministic episode to get final gains
                from pid_tuner.rl_agent import load_gains_from_checkpoint
                gains = load_gains_from_checkpoint(ckpt, self.store, axis=self._axis)
                self.root.after(0, self._on_rl_complete, gains)
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("RL Error", str(e)))

        threading.Thread(target=_run_train, daemon=True).start()

    def _on_rl_complete(self, gains: Dict[str, float]) -> None:
        self._refresh_all_rows()
        self._run_sim()
        gain_str = "\n".join(f"  {k} = {v:.5f}" for k, v in gains.items())
        messagebox.showinfo("RL Auto-Tune Complete",
                            f"Optimal gains applied:\n{gain_str}")

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Start the GUI event loop (blocking)."""
        self._worker.start()
        self.root.mainloop()

    def _on_close(self) -> None:
        self._worker.stop()
        self.root.destroy()


# ---------------------------------------------------------------------------
# Standalone entry
# ---------------------------------------------------------------------------

def main():
    import argparse
    parser = argparse.ArgumentParser(description="PID Tuner GUI")
    parser.add_argument("--axis", default="pitch",
                        choices=["pitch", "roll", "yaw"])
    parser.add_argument("--config", default=None,
                        help="Path to YAML config to pre-load")
    args = parser.parse_args()

    store = ParamStore()
    if args.config and os.path.isfile(args.config):
        store.load_yaml(args.config)
        print(f"[GUI] Loaded config: {args.config}")

    app = PIDTunerGUI(store=store, axis=args.axis)
    app.run()


if __name__ == "__main__":
    main()
