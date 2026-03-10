"""
px4_rate_controller.py  –  PX4 风格的角速率控制器

实现 PX4 固定翼的角速率控制架构：
- 使用 P + I + FF 结构（无 D 项）
- 支持运行时增益调整
- 三个独立通道：滚转、俯仰、偏航

参考：
- PX4 FW_RR_P, FW_RR_I, FW_RR_FF (滚转)
- PX4 FW_PR_P, FW_PR_I, FW_PR_FF (俯仰)
- PX4 FW_YR_P, FW_YR_I, FW_YR_FF (偏航)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from control.pid_controller import PX4StyleController


@dataclass
class RateOutput:
    """角速率控制器输出（归一化舵面偏转增量 -1..1）"""
    elevator: float = 0.0
    aileron:  float = 0.0
    rudder:   float = 0.0


class PX4RateController:
    """
    PX4 风格的角速率控制器
    
    使用 P+I+FF 结构，无 D 项。这种设计更适合固定翼飞机，
    因为前馈项提供快速响应，而不依赖噪声敏感的微分项。
    
    Parameters
    ----------
    dt : 默认时间步长（秒）
    """
    
    def __init__(self, dt: float = 0.01):
        self.dt = dt
        
        # 创建三个通道的控制器
        # 滚转通道（Roll）
        self.roll_controller = PX4StyleController(
            kp=0.06,   # FW_RR_P 典型值
            ki=0.01,   # FW_RR_I 典型值
            kff=0.4,   # FW_RR_FF 典型值
            tc=0.5,    # FW_R_TC 典型值
            output_min=-1.0,
            output_max=1.0,
            dt=dt
        )
        
        # 俯仰通道（Pitch）
        self.pitch_controller = PX4StyleController(
            kp=0.04,   # FW_PR_P 典型值
            ki=0.01,   # FW_PR_I 典型值
            kff=0.4,   # FW_PR_FF 典型值
            tc=0.5,    # FW_P_TC 典型值
            output_min=-1.0,
            output_max=1.0,
            dt=dt
        )
        
        # 偏航通道（Yaw）
        self.yaw_controller = PX4StyleController(
            kp=0.05,   # FW_YR_P 典型值
            ki=0.01,   # FW_YR_I 典型值
            kff=0.3,   # FW_YR_FF 典型值
            tc=0.5,    # FW_Y_TC 典型值
            output_min=-1.0,
            output_max=1.0,
            dt=dt
        )
    
    def update(
        self,
        p: float, q: float, r: float,              # 实际角速率 (rad/s)
        p_cmd: float, q_cmd: float, r_cmd: float,  # 期望角速率 (rad/s)
        dt: float = None,
    ) -> RateOutput:
        """
        计算舵面偏转增量
        
        Parameters
        ----------
        p, q, r       : 实际机体角速率 (rad/s)
        p_cmd, q_cmd, r_cmd : 期望角速率（来自姿态控制器）
        dt            : 时间步长（秒）
        
        Returns
        -------
        RateOutput : 归一化舵面偏转增量
        """
        if dt is None:
            dt = self.dt
        
        # 滚转通道 → 副翼
        aileron = self.roll_controller.update(
            error=p_cmd - p,
            setpoint=p_cmd,
            dt=dt
        )
        
        # 俯仰通道 → 升降舵
        elevator = self.pitch_controller.update(
            error=q_cmd - q,
            setpoint=q_cmd,
            dt=dt
        )
        
        # 偏航通道 → 方向舵
        rudder = self.yaw_controller.update(
            error=r_cmd - r,
            setpoint=r_cmd,
            dt=dt
        )
        
        return RateOutput(
            elevator=elevator,
            aileron=aileron,
            rudder=rudder
        )
    
    def set_gains(
        self,
        channel: str,  # 'roll', 'pitch', 'yaw'
        kp: float = None,
        ki: float = None,
        kff: float = None,
        tc: float = None
    ) -> None:
        """
        运行时更新指定通道的增益
        
        Parameters
        ----------
        channel : 通道名称 ('roll', 'pitch', 'yaw')
        kp, ki, kff, tc : 要更新的增益（None 表示不更新）
        """
        if channel == 'roll':
            self.roll_controller.set_gains(kp, ki, kff, tc)
        elif channel == 'pitch':
            self.pitch_controller.set_gains(kp, ki, kff, tc)
        elif channel == 'yaw':
            self.yaw_controller.set_gains(kp, ki, kff, tc)
        else:
            raise ValueError(f"Unknown channel: {channel}. Must be 'roll', 'pitch', or 'yaw'.")
    
    def set_all_gains(
        self,
        roll_gains: dict = None,
        pitch_gains: dict = None,
        yaw_gains: dict = None
    ) -> None:
        """
        批量更新所有通道的增益
        
        Parameters
        ----------
        roll_gains  : {'kp': ..., 'ki': ..., 'kff': ..., 'tc': ...}
        pitch_gains : {'kp': ..., 'ki': ..., 'kff': ..., 'tc': ...}
        yaw_gains   : {'kp': ..., 'ki': ..., 'kff': ..., 'tc': ...}
        """
        if roll_gains:
            self.set_gains('roll', **roll_gains)
        if pitch_gains:
            self.set_gains('pitch', **pitch_gains)
        if yaw_gains:
            self.set_gains('yaw', **yaw_gains)
    
    def reset(self) -> None:
        """重置所有控制器（模式切换时调用）"""
        self.roll_controller.reset()
        self.pitch_controller.reset()
        self.yaw_controller.reset()
    
    def __repr__(self) -> str:
        return (f"PX4RateController(\n"
                f"  roll:  {self.roll_controller}\n"
                f"  pitch: {self.pitch_controller}\n"
                f"  yaw:   {self.yaw_controller}\n"
                f")")
