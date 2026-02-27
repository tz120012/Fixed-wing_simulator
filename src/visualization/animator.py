"""
animator.py  –  3D real-time trajectory animation (Matplotlib FuncAnimation).

Mirrors the style of Quadcopter_SimCon/Simulation/utils/animation.py,
adapted for fixed-wing aircraft body outline (fuselage + wings).
"""

from __future__ import annotations

import numpy as np
from typing import Dict, Optional


class FixedWingAnimator:
    """
    3D trajectory animator using Matplotlib FuncAnimation.

    Features:
      - Simple fixed-wing silhouette (fuselage line + wing lines)
      - Actual trajectory trace (blue)
      - Desired trajectory trace (red dashed)
      - Waypoints (green scatter)
    """

    def animate(
        self,
        history:  Dict[str, np.ndarray],
        uav_name: str = "UAV",
        num_frames: int = 8,    # update every N simulation steps
        show: bool = True,
        save_path: Optional[str] = None,
    ) -> None:
        """
        Create and display (or save) the 3D animation.

        Parameters
        ----------
        history    : StateHistory.to_dict() output
        uav_name   : display name
        num_frames : animation stride (update interval)
        show       : call plt.show() at the end
        save_path  : if provided, save GIF to this path
        """
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        t   = history["t"]
        xN  = history["x_north"]
        xE  = history["x_east"]
        alt = history["altitude"]
        phi   = history["phi"]
        theta = history["theta"]
        psi   = history["psi"]

        has_des = "des_north" in history and not np.all(history.get("des_north", [0]) == 0)

        # --- Figure setup ---------------------------------------------------
        fig = plt.figure(figsize=(10, 8))
        ax  = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_zlabel("Alt (m)")
        ax.set_title(f"{uav_name} – 3D Trajectory Animation")

        # Plot full desired trajectory (static)
        if has_des:
            ax.plot(history["des_east"], history["des_north"], -history["des_down"],
                    "r--", linewidth=1, alpha=0.5, label="Desired")

        # Actual trajectory trace (grows over time)
        trace_line, = ax.plot([], [], [], "b-", linewidth=1.5, label="Actual")

        # Aircraft body lines (fuselage + wings)
        body_line,  = ax.plot([], [], [], "k-",  linewidth=2.5)
        wing_line,  = ax.plot([], [], [], "k-",  linewidth=1.5)
        htail_line, = ax.plot([], [], [], "k--", linewidth=1.0)

        ax.legend(loc="upper left", fontsize=8)

        # --- Body geometry in body frame ------------------------------------
        # Fuselage: ±2 m along x-body
        fuse   = np.array([[2.0, 0, 0], [-2.0, 0, 0]]).T
        # Wings:   ±5 m along y-body at x=0
        wings  = np.array([[0, 5.0, 0], [0, -5.0, 0]]).T
        # H-tail: ±1.5 m at x=-2
        htail  = np.array([[-2, 1.5, 0], [-2, -1.5, 0]]).T

        def _rot(ph, th, ps):
            """Rotation matrix body→NED."""
            cp, sp = np.cos(ph), np.sin(ph)
            ct, st = np.cos(th), np.sin(th)
            cs, ss = np.cos(ps), np.sin(ps)
            return np.array([
                [ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss],
                [ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs],
                [-st,   sp*ct,            cp*ct           ],
            ])

        # Auto-scale axes once
        margin = 20.0
        ax.set_xlim(xE.min() - margin, xE.max() + margin)
        ax.set_ylim(xN.min() - margin, xN.max() + margin)
        ax.set_zlim(max(0, alt.min() - 20), alt.max() + margin)

        # Pre-compute frames
        frame_indices = list(range(0, len(t), num_frames))

        def update(frame_idx: int):
            i = frame_indices[frame_idx]

            # History trace
            trace_line.set_data(xE[:i+1], xN[:i+1])
            trace_line.set_3d_properties(alt[:i+1])

            # Aircraft body
            R   = _rot(phi[i], theta[i], psi[i])
            pos = np.array([xE[i], xN[i], alt[i]])

            def _pts(seg_body):
                pts_ned = R @ seg_body  # (3, 2)
                # rotate to plot frame (x=E, y=N, z=alt)
                return pos[0] + pts_ned[1], pos[1] + pts_ned[0], pos[2] - pts_ned[2]

            bx, by, bz = _pts(fuse)
            body_line.set_data(bx, by); body_line.set_3d_properties(bz)

            wx, wy, wz = _pts(wings)
            wing_line.set_data(wx, wy); wing_line.set_3d_properties(wz)

            hx, hy, hz = _pts(htail)
            htail_line.set_data(hx, hy); htail_line.set_3d_properties(hz)

            ax.set_title(f"{uav_name}  t={t[i]:.1f}s  "
                         f"alt={alt[i]:.0f}m  V={history['airspeed'][i]:.1f}m/s")

            return trace_line, body_line, wing_line, htail_line

        ani = animation.FuncAnimation(
            fig, update,
            frames=len(frame_indices),
            interval=40,   # ms per frame (~25 fps)
            blit=False,
        )

        if save_path:
            ani.save(save_path, writer="pillow", fps=25)
            print(f"[Animator] Saved to {save_path}")

        if show:
            plt.show()
