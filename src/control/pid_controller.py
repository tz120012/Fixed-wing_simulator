"""
pid_controller.py  –  Generic PID controller with anti-windup.

Design mirrors ArduPilot's AC_PID implementation:
  - Proportional, integral, derivative terms
  - Clamping-based anti-windup (integral accumulates only when unsaturated)
  - Optional first-order derivative low-pass filter
  - reset() for mode transitions
"""

from __future__ import annotations

import numpy as np
from utils.math_utils import saturate


class PIDController:
    """
    Discrete-time PID controller with clamping anti-windup.

    Parameters
    ----------
    kp, ki, kd   : P/I/D gains
    output_min   : lower saturation limit
    output_max   : upper saturation limit
    d_lpf_hz     : derivative low-pass filter cutoff (Hz); 0 = no filter
    dt           : default time step (s); can be overridden in update()
    """

    def __init__(
        self,
        kp:         float = 1.0,
        ki:         float = 0.0,
        kd:         float = 0.0,
        output_min: float = -1.0,
        output_max: float =  1.0,
        d_lpf_hz:   float = 20.0,
        dt:         float = 0.01,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.d_lpf_hz   = d_lpf_hz
        self.dt_default = dt

        self._integral:  float = 0.0
        self._prev_error: float = 0.0
        self._d_filtered: float = 0.0
        self._saturated:  bool  = False

    # ------------------------------------------------------------------

    def update(self, error: float, dt: float = None, feed_forward: float = 0.0) -> float:
        """
        Compute PID output for *error* at this timestep.

        Parameters
        ----------
        error        : set-point minus measured value
        dt           : time step (s); uses default if None
        feed_forward : optional feed-forward term added to output

        Returns
        -------
        output : float  (clamped to [output_min, output_max])
        """
        if dt is None or dt <= 0.0:
            dt = self.dt_default

        # --- Derivative (with optional LPF) ---------------------------------
        d_raw = (error - self._prev_error) / dt
        if self.d_lpf_hz > 0.0:
            alpha = 1.0 / (1.0 + 1.0 / (2.0 * np.pi * self.d_lpf_hz * dt))
            self._d_filtered = alpha * self._d_filtered + (1.0 - alpha) * d_raw
        else:
            self._d_filtered = d_raw

        # --- Proportional ----------------------------------------------------
        p_out = self.kp * error

        # --- Integral (clamping anti-windup) ---------------------------------
        # Only integrate when not saturated (ArduPilot clamping method)
        if not self._saturated:
            self._integral += self.ki * error * dt
            # Clamp integral itself to output range
            self._integral = saturate(self._integral, self.output_min, self.output_max)

        # --- Total output ----------------------------------------------------
        output_raw = p_out + self._integral + self.kd * self._d_filtered + feed_forward

        # --- Saturation & anti-windup flag -----------------------------------
        output = saturate(output_raw, self.output_min, self.output_max)
        self._saturated = (output_raw != output)

        self._prev_error = error
        return output

    def reset(self, zero_integrator: bool = True) -> None:
        """Reset controller state (call on mode transitions)."""
        self._prev_error  = 0.0
        self._d_filtered  = 0.0
        self._saturated   = False
        if zero_integrator:
            self._integral = 0.0

    def set_gains(self, kp: float = None, ki: float = None, kd: float = None) -> None:
        """Update gains at runtime."""
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd

    def __repr__(self) -> str:
        return (f"PIDController(kp={self.kp}, ki={self.ki}, kd={self.kd}, "
                f"out=[{self.output_min},{self.output_max}])")


class PX4StyleController:
    """
    PX4 风格的角速率控制器：P + I + FF（无D项）
    
    控制律：
    output = P * error + I * ∫error + FF * setpoint
    
    这种设计更适合固定翼飞机的角速率控制，因为：
    1. 前馈（FF）提供快速响应
    2. 比例（P）提供误差修正
    3. 积分（I）消除稳态误差
    4. 不使用微分（D）避免高频噪声敏感性
    
    Parameters
    ----------
    kp         : 比例增益
    ki         : 积分增益
    kff        : 前馈增益
    tc         : 时间常数（用于外环姿态控制）
    output_min : 输出下限
    output_max : 输出上限
    dt         : 默认时间步长（秒）
    """
    
    def __init__(
        self,
        kp: float = 0.06,
        ki: float = 0.01,
        kff: float = 0.4,
        tc: float = 0.5,
        output_min: float = -1.0,
        output_max: float = 1.0,
        dt: float = 0.01,
    ):
        self.kp = kp
        self.ki = ki
        self.kff = kff
        self.tc = tc
        self.output_min = output_min
        self.output_max = output_max
        self.dt_default = dt
        
        self._integral = 0.0
        self._saturated = False
    
    def update(
        self, 
        error: float, 
        setpoint: float,
        dt: float = None
    ) -> float:
        """
        更新控制器输出
        
        Parameters
        ----------
        error    : 误差（期望值 - 实际值）
        setpoint : 期望值（用于前馈）
        dt       : 时间步长（秒），None 则使用默认值
        
        Returns
        -------
        output : float，控制输出（已限幅）
        """
        if dt is None or dt <= 0.0:
            dt = self.dt_default
        
        # 比例项
        p_out = self.kp * error
        
        # 积分项（带抗饱和）
        if not self._saturated:
            self._integral += self.ki * error * dt
            # 限制积分项本身
            self._integral = saturate(
                self._integral, 
                self.output_min, 
                self.output_max
            )
        
        # 前馈项
        ff_out = self.kff * setpoint
        
        # 总输出
        output_raw = p_out + self._integral + ff_out
        output = saturate(output_raw, self.output_min, self.output_max)
        
        # 更新饱和标志
        self._saturated = (output_raw != output)
        
        return output
    
    def reset(self, zero_integrator: bool = True) -> None:
        """重置控制器状态（模式切换时调用）"""
        self._saturated = False
        if zero_integrator:
            self._integral = 0.0
    
    def set_gains(
        self, 
        kp: float = None, 
        ki: float = None, 
        kff: float = None,
        tc: float = None
    ) -> None:
        """运行时更新增益"""
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kff is not None: self.kff = kff
        if tc is not None: self.tc = tc
    
    def __repr__(self) -> str:
        return (f"PX4StyleController(kp={self.kp}, ki={self.ki}, kff={self.kff}, "
                f"tc={self.tc}, out=[{self.output_min},{self.output_max}])")
