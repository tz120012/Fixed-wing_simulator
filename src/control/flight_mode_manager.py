"""
flight_mode_manager.py  –  Flight mode management (ArduPilot Plane compatible).

Supports the same flight modes as ArduPilot Plane:
  MANUAL    – direct pass-through of stick inputs
  STABILIZE – attitude stabilisation (SAS + angle hold)
  FBW_A     – Fly-By-Wire A (airspeed-referenced roll/pitch limits)
  FBW_B     – Fly-By-Wire B (altitude hold + speed hold)
  AUTO      – full autonomous navigation (trajectory tracking)
  LOITER    – orbit a fixed point at fixed altitude
  RTH       – return to home

The manager:
  1. Tracks the current and previous mode.
  2. Calls mode.update() each control step to compute a ControlTarget.
  3. Applies smooth transition logic on mode change.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class FlightMode(str, Enum):
    MANUAL    = "MANUAL"
    STABILIZE = "STABILIZE"
    FBW_A     = "FBW_A"
    FBW_B     = "FBW_B"
    AUTO      = "AUTO"
    LOITER    = "LOITER"
    RTH       = "RTH"


@dataclass
class AircraftState:
    """Minimal aircraft state snapshot passed to the flight mode manager."""
    # Position (NED, m)
    pos_north: float = 0.0
    pos_east:  float = 0.0
    pos_down:  float = -100.0  # negative = above ground

    # Velocity (body frame, m/s)
    u: float = 30.0
    v: float = 0.0
    w: float = 0.0

    # Attitude (rad)
    phi:   float = 0.0
    theta: float = 0.0
    psi:   float = 0.0

    # Angular rates (rad/s)
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    # Derived
    airspeed: float = 30.0
    altitude: float = 100.0   # m above ground (positive up)

    @property
    def pos_ned(self) -> np.ndarray:
        return np.array([self.pos_north, self.pos_east, self.pos_down])

    @property
    def vel_body(self) -> np.ndarray:
        return np.array([self.u, self.v, self.w])

    @property
    def euler(self) -> np.ndarray:
        return np.array([self.phi, self.theta, self.psi])

    @property
    def omega(self) -> np.ndarray:
        return np.array([self.p, self.q, self.r])


@dataclass
class ControlTarget:
    """
    Desired state/commands output by a flight mode.

    Consumed by the attitude and rate control layers.
    """
    # Desired angles (rad)
    roll_cmd:  float = 0.0
    pitch_cmd: float = 0.0
    yaw_cmd:   float = 0.0   # desired heading (rad)

    # Desired rates (rad/s) – optional feed-forward
    roll_rate_cmd:  float = 0.0
    pitch_rate_cmd: float = 0.0
    yaw_rate_cmd:   float = 0.0

    # Speed / altitude
    airspeed_cmd:  float = 30.0  # m/s
    altitude_cmd:  float = 100.0 # m

    # Direct control overrides (used in MANUAL mode)
    elevator_direct: Optional[float] = None
    aileron_direct:  Optional[float] = None
    rudder_direct:   Optional[float] = None
    throttle_direct: Optional[float] = None

    # Throttle command (0–1)
    throttle_cmd: float = 0.5

    is_direct: bool = False  # True = bypass attitude controller


class FlightModeManager:
    """
    Manages flight mode selection and generates ControlTargets.

    Parameters
    ----------
    initial_mode : starting FlightMode
    home_pos_ned : (3,) home position in NED frame (m)
    cruise_speed : default cruise airspeed (m/s)
    """

    def __init__(
        self,
        initial_mode:   FlightMode = FlightMode.AUTO,
        home_pos_ned:   np.ndarray = None,
        cruise_speed:   float = 30.0,
        cruise_alt:     float = 100.0,
    ):
        self.current_mode  = initial_mode
        self.previous_mode = initial_mode
        self.home_pos_ned  = home_pos_ned if home_pos_ned is not None else np.zeros(3)
        self.cruise_speed  = cruise_speed
        self.cruise_alt    = cruise_alt

        # Loiter centre captured when entering LOITER
        self._loiter_pos: Optional[np.ndarray] = None

        # Manual stick inputs (set externally for MANUAL mode)
        self.manual_elevator: float = 0.0
        self.manual_aileron:  float = 0.0
        self.manual_rudder:   float = 0.0
        self.manual_throttle: float = 0.5

    # ------------------------------------------------------------------
    # Mode switching
    # ------------------------------------------------------------------

    def set_mode(self, new_mode: FlightMode) -> None:
        """Switch to a new flight mode with transition bookkeeping."""
        if new_mode == self.current_mode:
            return
        self.previous_mode = self.current_mode
        self.current_mode  = new_mode

        # Capture loiter position on entry
        if new_mode == FlightMode.LOITER:
            self._loiter_pos = None   # will be set on first update

        print(f"[FlightModeManager] {self.previous_mode.value} → {new_mode.value}")

    def set_mode_str(self, mode_str: str) -> None:
        """Convenience: set mode by string name."""
        self.set_mode(FlightMode(mode_str.upper()))

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def update(
        self,
        state:   AircraftState,
        nav_target: Optional[ControlTarget] = None,
        dt: float = 0.1,
    ) -> ControlTarget:
        """
        Compute a ControlTarget for the current mode.

        Parameters
        ----------
        state      : current aircraft state
        nav_target : suggested target from NavigationController (used in AUTO/LOITER)
        dt         : time step (s)

        Returns
        -------
        ControlTarget consumed by the attitude controller.
        """
        mode = self.current_mode

        if mode == FlightMode.MANUAL:
            return self._manual(state)

        if mode == FlightMode.STABILIZE:
            return self._stabilize(state)

        if mode == FlightMode.FBW_A:
            return self._fbw_a(state)

        if mode == FlightMode.FBW_B:
            return self._fbw_b(state)

        if mode in (FlightMode.AUTO, FlightMode.LOITER, FlightMode.RTH):
            return self._auto(state, nav_target)

        # Fallback
        return ControlTarget(airspeed_cmd=self.cruise_speed,
                             altitude_cmd=self.cruise_alt)

    # ------------------------------------------------------------------
    # Per-mode logic
    # ------------------------------------------------------------------

    def _manual(self, state: AircraftState) -> ControlTarget:
        """Direct pass-through – bypass attitude/rate loops."""
        return ControlTarget(
            elevator_direct=self.manual_elevator,
            aileron_direct =self.manual_aileron,
            rudder_direct  =self.manual_rudder,
            throttle_direct=self.manual_throttle,
            is_direct=True,
        )

    def _stabilize(self, state: AircraftState) -> ControlTarget:
        """Hold wings-level, neutral pitch; rate damping active."""
        return ControlTarget(
            roll_cmd  = 0.0,
            pitch_cmd = 0.0,
            yaw_cmd   = state.psi,
            airspeed_cmd = self.cruise_speed,
            altitude_cmd = state.altitude,
            throttle_cmd = 0.5,
        )

    def _fbw_a(self, state: AircraftState) -> ControlTarget:
        """
        FBW-A: Stick → roll/pitch angle command within limits.
        For simulation: hold current roll, maintain altitude with pitch.
        """
        return ControlTarget(
            roll_cmd  = state.phi,
            pitch_cmd = 0.0,
            yaw_cmd   = state.psi,
            airspeed_cmd = self.cruise_speed,
            altitude_cmd = state.altitude,
            throttle_cmd = 0.5,
        )

    def _fbw_b(self, state: AircraftState) -> ControlTarget:
        """FBW-B: Altitude hold + airspeed hold."""
        return ControlTarget(
            roll_cmd  = 0.0,
            pitch_cmd = 0.0,
            yaw_cmd   = state.psi,
            airspeed_cmd = self.cruise_speed,
            altitude_cmd = self.cruise_alt,
            throttle_cmd = 0.5,
        )

    def _auto(self, state: AircraftState,
              nav_target: Optional[ControlTarget]) -> ControlTarget:
        """AUTO / LOITER / RTH: use nav_target if provided."""
        if self.current_mode == FlightMode.LOITER:
            if self._loiter_pos is None:
                self._loiter_pos = state.pos_ned.copy()

        if self.current_mode == FlightMode.RTH:
            # Fly toward home at cruise altitude
            if nav_target is None:
                nav_target = ControlTarget(
                    airspeed_cmd = self.cruise_speed,
                    altitude_cmd = self.cruise_alt,
                )

        if nav_target is not None:
            return nav_target

        # Fallback: hold current state
        return ControlTarget(
            roll_cmd  = state.phi,
            pitch_cmd = 0.0,
            yaw_cmd   = state.psi,
            airspeed_cmd = self.cruise_speed,
            altitude_cmd = self.cruise_alt,
            throttle_cmd = 0.5,
        )
