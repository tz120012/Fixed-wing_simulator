"""
px4_attitude_controller.py  –  PX4 风格的姿态控制器

实现 PX4 固定翼的姿态控制架构：
- 使用时间常数（TC）调节响应速度
- 姿态误差 / TC = 期望角速率
- 三个独立通道：滚转、俯仰、偏航

参考：
- PX4 FW_R_TC (滚转时间常数)
- PX4 FW_P_TC (俯仰时间常数)
- PX4 FW_Y_TC (偏航时间常数)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from utils.math_utils import wrap_angle


@dataclass
class AttitudeOutput:
    """姿态控制器输出（期望角速率 rad/s）"""
    roll_rate_cmd:  float = 0.0
    pitch_rate_cmd: float = 0.0
    yaw_rate_cmd:   float = 0.0


class PX4AttitudeController:
    """
    PX4 风格的姿态控制器
    
    使用时间常数（TC）来调节响应速度：
    - TC 越小 → 响应越快，越硬
    - TC 越大 → 响应越慢，越软
    
    控制律：rate_cmd = attitude_error / TC
    
    Parameters
    ----------
    dt : 默认时间步长（秒）
    """
    
    def __init__(self, dt: float = 0.01):
        self.dt = dt
        
        # 时间常数（秒）- PX4 默认值
        self.roll_tc = 0.5   # FW_R_TC
        self.pitch_tc = 0.5  # FW_P_TC
        self.yaw_tc = 0.5    # FW_Y_TC
        
        # 角速率限制（rad/s）
        self.max_roll_rate = np.radians(120.0)   # 120 deg/s
        self.max_pitch_rate = np.radians(60.0)   # 60 deg/s
        self.max_yaw_rate = np.radians(45.0)     # 45 deg/s
    
    def update(
        self,
        phi: float, theta: float, psi: float,        # 实际姿态 (rad)
        roll_cmd: float, pitch_cmd: float, yaw_cmd: float,  # 期望姿态 (rad)
        dt: float = None,
    ) -> AttitudeOutput:
        """
        计算期望角速率
        
        Parameters
        ----------
        phi, theta, psi     : 实际欧拉角 (rad)
        roll_cmd, pitch_cmd, yaw_cmd : 期望欧拉角 (rad)
        dt                  : 时间步长（秒）
        
        Returns
        -------
        AttitudeOutput : 期望角速率指令
        """
        if dt is None:
            dt = self.dt
        
        # 计算姿态误差（wrap 到 ±π）
        roll_err = wrap_angle(roll_cmd - phi)
        pitch_err = wrap_angle(pitch_cmd - theta)
        yaw_err = wrap_angle(yaw_cmd - psi)
        
        # 使用时间常数计算期望角速率
        # rate_cmd = error / TC
        p_cmd = np.clip(
            roll_err / self.roll_tc,
            -self.max_roll_rate,
            self.max_roll_rate
        )
        
        q_cmd = np.clip(
            pitch_err / self.pitch_tc,
            -self.max_pitch_rate,
            self.max_pitch_rate
        )
        
        r_cmd = np.clip(
            yaw_err / self.yaw_tc,
            -self.max_yaw_rate,
            self.max_yaw_rate
        )
        
        return AttitudeOutput(
            roll_rate_cmd=p_cmd,
            pitch_rate_cmd=q_cmd,
            yaw_rate_cmd=r_cmd
        )
    
    def set_time_constants(
        self,
        roll_tc: float = None,
        pitch_tc: float = None,
        yaw_tc: float = None
    ) -> None:
        """运行时更新时间常数"""
        if roll_tc is not None: 
            self.roll_tc = max(0.1, roll_tc)  # 最小 0.1 秒
        if pitch_tc is not None: 
            self.pitch_tc = max(0.1, pitch_tc)
        if yaw_tc is not None: 
            self.yaw_tc = max(0.1, yaw_tc)
    
    def reset(self) -> None:
        """重置控制器（当前实现无状态，保留接口一致性）"""
        pass
    
    def __repr__(self) -> str:
        return (f"PX4AttitudeController(\n"
                f"  roll_tc={self.roll_tc:.3f}s, "
                f"  pitch_tc={self.pitch_tc:.3f}s, "
                f"  yaw_tc={self.yaw_tc:.3f}s\n"
                f")")
