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

import os
import sys
import threading
from typing import Dict, List, Optional

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
    One-shot simulation worker: runs a full 6-second episode (600 steps at dt=0.01)
    in a background thread using the current ParamStore gains, then fires an
    optional callback with the completed buffer.

    Usage
    -----
    worker.run_once(on_done=callback)   # starts background thread
    worker.abort()                      # cancel in-progress run
    """

    _EPISODE_STEPS = 600   # 6 s at dt=0.01
    _MAX_HISTORY   = 3     # how many previous curves to keep for overlay

    def __init__(self, store: ParamStore, axis: str = "pitch"):
        self.store = store
        self.axis  = axis
        self._env  = PIDTuningEnv(axis=axis, episode_steps=self._EPISODE_STEPS, dt=0.01)
        self._lock = threading.Lock()

        self._buf: List[Dict]           = []
        self._history: List[List[Dict]] = []
        self._metrics: Dict[str, float] = {}

        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run_once(self, on_done=None) -> None:
        """Run a full episode in a background thread.

        Parameters
        ----------
        on_done : callable(buf, metrics) | None
            Called in the background thread when the episode finishes.
            The GUI should schedule GUI updates via ``root.after(0, ...)``.
        """
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run, args=(on_done,), daemon=True)
        self._thread.start()

    def abort(self) -> None:
        """Abort a running episode (no-op if idle)."""
        self._running = False

    def change_axis(self, axis: str) -> None:
        self.abort()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self.axis = axis
        self._env = PIDTuningEnv(axis=axis, episode_steps=self._EPISODE_STEPS, dt=0.01)
        with self._lock:
            self._buf.clear()
            self._history.clear()
        self._metrics = {}

    def get_buffer(self) -> List[Dict]:
        with self._lock:
            return list(self._buf)

    def get_history(self) -> List[List[Dict]]:
        """Return snapshots of previous completed curves (for overlay)."""
        with self._lock:
            return [list(snap) for snap in self._history]

    def get_metrics(self) -> Dict[str, float]:
        return dict(self._metrics)

    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _apply_gains(self) -> None:
        """Write current ParamStore gains into env cascaded plant (no randomisation)."""
        gains = self.store.get_all()
        gr    = _GAIN_RANGES[self.axis]

        # Outer-loop (attitude) gains
        att_gain_map = {
            "pitch": ("PTCH_P",  "PTCH_D"),
            "roll":  ("ROLL_P",  "ROLL_D"),
            "yaw":   ("YAW_P",   None),
        }
        kp_att_key, kd_att_key = att_gain_map.get(self.axis, att_gain_map["pitch"])
        kp_att = float(gains.get(kp_att_key, self._env._kp_att))
        kd_att = float(gains.get(kd_att_key, self._env._kd_att)) if kd_att_key else 0.0
        self._env._kp_att = kp_att
        self._env._kd_att = kd_att
        self._env._plant.set_att_gains(kp_att, kd_att)

        # Inner-loop (rate) gains + FF
        rate_gain_map = {
            "pitch": ("PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF"),
            "roll":  ("ROLL_RATE_P",  "ROLL_RATE_I",  "ROLL_D",      "ROLL_RATE_FF"),
            "yaw":   ("YAW_RATE_P",   "YAW_RATE_I",   None,          None),
        }
        kp_key, ki_key, kd_key, kff_key = rate_gain_map.get(self.axis, rate_gain_map["pitch"])
        self._env._kp = float(np.clip(gains.get(kp_key, self._env._kp), *gr["kp"]))
        self._env._ki = float(np.clip(gains.get(ki_key, self._env._ki), *gr["ki"]))
        self._env._kd = float(np.clip(
            gains.get(kd_key, self._env._kd) if kd_key else 0.0, *gr["kd"]))
        kff = float(gains.get(kff_key, 0.0)) if kff_key else 0.0
        self._env._pid.kp = self._env._kp
        self._env._pid.ki = self._env._ki
        self._env._pid.kd = self._env._kd
        self._env._plant.set_rate_gains(self._env._kp, self._env._ki, self._env._kd, kff)

    def _run(self, on_done) -> None:
        # Save previous buffer to history (if non-empty)
        with self._lock:
            if len(self._buf) >= 10:
                self._history.append(list(self._buf))
                if len(self._history) > self._MAX_HISTORY:
                    self._history.pop(0)
            self._buf.clear()

        # Apply current GUI gains, then reset env (t → 0, integrator cleared)
        self._apply_gains()
        obs, _ = self._env.reset()
        # reset() may randomise rate gains; restore ours
        self._env._plant.set_att_gains(self._env._kp_att, self._env._kd_att)
        self._env._plant.set_rate_gains(self._env._kp, self._env._ki, self._env._kd,
                                        self._env._plant.kff_rate)

        action = np.zeros(3)
        new_buf: List[Dict] = []
        info: Dict = {}

        for _ in range(self._EPISODE_STEPS):
            if not self._running:
                break   # aborted

            obs, reward, term, trunc, info = self._env.step(action)

            # Rate-loop PID terms (from inner rate PID in _CascadedPlant)
            rate_pid = self._env._plant._rate_pid
            p_term = rate_pid.kp * (info["rate_ref"] - info["rate"])
            i_term = float(rate_pid._integral)
            d_term = 0.0   # finite-diff D is internal to _MiniPID

            new_buf.append({
                "t":        self._env._t,
                "ref":      info["ref"],           # attitude setpoint
                "output":   info["output"],        # attitude (outer output)
                "rate_ref": info["rate_ref"],      # rate setpoint (outer→inner)
                "rate":     info["rate"],          # actual angular rate
                "error":    info["error"],         # attitude error
                "p_term":   p_term,
                "i_term":   i_term,
                "d_term":   d_term,
            })

            if term or trunc:
                break

        # Commit results
        with self._lock:
            self._buf = new_buf

        self._metrics = {
            "overshoot":   info.get("peak_overshoot", 0.0) * 100.0,
            "settle_time": info.get("settle_time", 0.0),
            "iae":         info.get("integral_abs_error", 0.0),
        }

        self._running = False
        if on_done is not None:
            on_done(list(new_buf), dict(self._metrics))


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
        self._btn_run = ttk.Button(btn_frame, text="▶ Run",  command=self._run_sim,
                                   style="Green.TButton")
        self._btn_run.pack(side="left", padx=2)
        self._btn_stop = ttk.Button(btn_frame, text="■ Stop", command=self._stop_sim,
                                    style="Red.TButton")
        self._btn_stop.pack(side="left", padx=2)
        self._btn_stop.state(["disabled"])
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
        fig = Figure(figsize=(8, 7), dpi=90,
                     facecolor=COLORS["bg"])
        self._fig = fig

        # Four subplots: attitude / rate / error / PID terms
        self._ax_att   = fig.add_subplot(4, 1, 1)   # attitude response
        self._ax_rate  = fig.add_subplot(4, 1, 2)   # angular rate response
        self._ax_error = fig.add_subplot(4, 1, 3)   # attitude error
        self._ax_pid   = fig.add_subplot(4, 1, 4)   # rate PID terms
        fig.subplots_adjust(hspace=0.55, left=0.08, right=0.97,
                            top=0.96, bottom=0.06)

        for ax in (self._ax_att, self._ax_rate, self._ax_error, self._ax_pid):
            ax.set_facecolor(COLORS["panel"])
            ax.tick_params(colors=COLORS["muted"], labelsize=7)
            for spine in ax.spines.values():
                spine.set_color(COLORS["muted"])
                spine.set_linewidth(0.5)

        self._ax_att.set_title("Attitude Response", color=COLORS["text"], fontsize=9)
        self._ax_rate.set_title("Angular Rate Response", color=COLORS["text"], fontsize=9)
        self._ax_error.set_title("Attitude Error", color=COLORS["text"], fontsize=9)
        self._ax_pid.set_title("Rate PID Terms", color=COLORS["text"], fontsize=9)

        canvas = FigureCanvasTkAgg(fig, master=self._right)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas_widget = canvas

        # Metrics label bar
        self._metrics_var = tk.StringVar(value="Overshoot: --   Settle: --   IAE: --")
        tk.Label(self._right, textvariable=self._metrics_var,
                 bg=COLORS["bg"], fg=COLORS["yellow"],
                 font=("Consolas", 9)).pack(side="bottom", pady=4)

    # ------------------------------------------------------------------
    # Chart drawing (called once per Run completion)
    # ------------------------------------------------------------------

    def _draw_charts(self, buf: List[Dict], metrics: Dict[str, float]) -> None:
        """Render completed episode curves. Called from GUI thread."""
        history = self._worker.get_history()

        for ax in (self._ax_att, self._ax_rate, self._ax_error, self._ax_pid):
            ax.cla()
            ax.set_facecolor(COLORS["panel"])
            ax.tick_params(colors=COLORS["muted"], labelsize=7)
            for spine in ax.spines.values():
                spine.set_color(COLORS["muted"])
                spine.set_linewidth(0.5)

        if len(buf) < 2:
            self._canvas_widget.draw_idle()
            return

        t_arr        = np.array([r["t"]        for r in buf])
        ref_arr      = np.array([r["ref"]      for r in buf])
        att_arr      = np.array([r["output"]   for r in buf])
        rate_ref_arr = np.array([r["rate_ref"] for r in buf])
        rate_arr     = np.array([r["rate"]     for r in buf])
        err_arr      = np.array([r["error"]    for r in buf])
        p_arr        = np.array([r["p_term"]   for r in buf])
        i_arr        = np.array([r["i_term"]   for r in buf])
        d_arr        = np.array([r["d_term"]   for r in buf])

        t_end = float(t_arr[-1]) if len(t_arr) else 6.0

        # ── History overlay (faded grey) ──
        n_hist = len(history)
        for i, snap in enumerate(history):
            if len(snap) < 2:
                continue
            alpha = 0.15 + 0.20 * (i / max(n_hist - 1, 1))
            ht   = np.array([r["t"]        for r in snap])
            ho   = np.array([r["output"]   for r in snap])
            hrr  = np.array([r["rate_ref"] for r in snap])
            hr   = np.array([r["rate"]     for r in snap])
            he   = np.array([r["error"]    for r in snap])
            hp   = np.array([r["p_term"]   for r in snap])
            hi_  = np.array([r["i_term"]   for r in snap])
            hd   = np.array([r["d_term"]   for r in snap])
            self._ax_att.plot(ht,  ho,  color="#888888", lw=0.8, alpha=alpha)
            self._ax_rate.plot(ht, hrr, color="#888888", lw=0.6, alpha=alpha)
            self._ax_rate.plot(ht, hr,  color="#666666", lw=0.8, alpha=alpha)
            self._ax_error.plot(ht, he, color="#888888", lw=0.8, alpha=alpha)
            self._ax_pid.plot(ht, hp,   color="#888888", lw=0.6, alpha=alpha)
            self._ax_pid.plot(ht, hi_,  color="#888888", lw=0.6, alpha=alpha)
            self._ax_pid.plot(ht, hd,   color="#888888", lw=0.6, alpha=alpha)

        # ── Current curves ──
        # 1. Attitude response
        self._ax_att.plot(t_arr, ref_arr, "--",
                          color=COLORS["yellow"], lw=1, label="Att Setpoint")
        self._ax_att.plot(t_arr, att_arr,
                          color=COLORS["accent"], lw=1.2, label="Attitude")
        self._ax_att.legend(fontsize=7, facecolor=COLORS["panel"],
                             labelcolor=COLORS["text"], loc="lower right")
        self._ax_att.set_title("Attitude Response", color=COLORS["text"], fontsize=9)
        self._ax_att.set_ylabel("(norm.)", color=COLORS["muted"], fontsize=7)

        # 2. Angular rate response
        self._ax_rate.plot(t_arr, rate_ref_arr, "--",
                           color=COLORS["yellow"], lw=1, label="Rate Setpoint")
        self._ax_rate.plot(t_arr, rate_arr,
                           color=COLORS["green"], lw=1.2, label="Rate")
        self._ax_rate.axhline(0, color=COLORS["muted"], lw=0.5, ls=":")
        self._ax_rate.legend(fontsize=7, facecolor=COLORS["panel"],
                              labelcolor=COLORS["text"], loc="lower right")
        self._ax_rate.set_title("Angular Rate Response", color=COLORS["text"], fontsize=9)
        self._ax_rate.set_ylabel("(norm.)", color=COLORS["muted"], fontsize=7)

        # 3. Attitude error
        self._ax_error.plot(t_arr, err_arr, color=COLORS["red"], lw=1)
        self._ax_error.axhline(0, color=COLORS["muted"], lw=0.5, ls="--")
        self._ax_error.set_title("Attitude Error", color=COLORS["text"], fontsize=9)

        # 4. Rate PID terms
        self._ax_pid.plot(t_arr, p_arr, color=COLORS["accent"], lw=1, label="P")
        self._ax_pid.plot(t_arr, i_arr, color=COLORS["green"],  lw=1, label="I")
        self._ax_pid.plot(t_arr, d_arr, color=COLORS["yellow"], lw=1, label="D")
        self._ax_pid.axhline(0, color=COLORS["muted"], lw=0.5, ls=":")
        self._ax_pid.legend(fontsize=7, facecolor=COLORS["panel"],
                             labelcolor=COLORS["text"], loc="upper right")
        self._ax_pid.set_title("Rate PID Terms", color=COLORS["text"], fontsize=9)

        # Fixed X-axis
        for ax in (self._ax_att, self._ax_rate, self._ax_error, self._ax_pid):
            ax.set_xlim(0.0, t_end)

        self._canvas_widget.draw_idle()

        if metrics:
            self._metrics_var.set(
                f"Overshoot: {metrics.get('overshoot', 0):.1f}%   "
                f"Settle: {metrics.get('settle_time', 0):.2f} s   "
                f"IAE: {metrics.get('iae', 0):.3f}"
            )

    # ------------------------------------------------------------------
    # Simulation control
    # ------------------------------------------------------------------

    def _run_sim(self) -> None:
        if self._worker.is_running():
            return
        # Disable Run, enable Stop, show status
        self._btn_run.state(["disabled"])
        self._btn_stop.state(["!disabled"])
        self._metrics_var.set("Running simulation …")

        def _on_done(buf, metrics):
            # Called from background thread → schedule GUI update on main thread
            self.root.after(0, self._on_sim_done, buf, metrics)

        self._worker.run_once(on_done=_on_done)

    def _on_sim_done(self, buf: List[Dict], metrics: Dict[str, float]) -> None:
        """Called in GUI thread when one-shot episode finishes."""
        self._btn_run.state(["!disabled"])
        self._btn_stop.state(["disabled"])
        self._draw_charts(buf, metrics)

    def _stop_sim(self) -> None:
        self._worker.abort()
        self._btn_run.state(["!disabled"])
        self._btn_stop.state(["disabled"])
        self._metrics_var.set("Stopped.")

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
        self.root.mainloop()

    def _on_close(self) -> None:
        self._worker.abort()
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
