"""
tecs_controller.py  –  Total Energy Control System (TECS)

Python port of ArduPilot AP_TECS (libraries/AP_TECS/AP_TECS.cpp).
Reference: Paul Riseborough 2013.

核心思想
--------
- **油门** 控制 **总比能量** (Specific Total Energy, STE = SPE + SKE)
- **俯仰角** 控制 **比能量分配比** (Specific Energy Balance, SEB = SPE·w_spe - SKE·w_ske)
- 高度与速度控制天然耦合，避免传统解耦PID中油门饱和与积分饱和问题

主要参数
--------
TECS_CLMB_MAX   最大爬升率 (m/s)          默认 5.0
TECS_SINK_MIN   最小下沉率 (m/s)          默认 2.0
TECS_SINK_MAX   最大下沉率 (m/s)          默认 5.0
TECS_TIME_CONST 控制时间常数 (s)          默认 5.0
TECS_THR_DAMP  油门阻尼                   默认 0.5
TECS_PTCH_DAMP 俯仰阻尼                   默认 0.3
TECS_INTEG_GAIN 积分器增益               默认 0.3
TECS_VERT_ACC   最大竖向加速度限制 (m/s²) 默认 7.0
TECS_SPDWEIGHT  速度权重 (0~2)            默认 1.0
TECS_RLL2THR   坡度转油门补偿增益         默认 10.0
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


def _saturate(val: float, lo: float, hi: float) -> float:
    return float(np.clip(val, lo, hi))


@dataclass
class TECSState:
    """TECS 输出状态（每次 update 后填充）."""
    throttle_dem: float = 0.5    # 归一化油门指令 [0, 1]
    pitch_dem:    float = 0.0    # 俯仰角指令 (rad)
    climb_rate:   float = 0.0    # 估计爬升率 (m/s)
    height:       float = 0.0    # 估计高度 (m)
    airspeed:     float = 0.0    # 估计真空速 (m/s)
    underspeed:   bool  = False  # 低速保护标志
    bad_descent:  bool  = False  # 不可达下沉保护标志


class TECSController:
    """
    Total Energy Control System — ArduPilot 风格 Python 实现。

    用法示例
    --------
    tecs = TECSController(
        max_climb_rate=5.0, min_sink_rate=2.0, max_sink_rate=5.0,
        time_const=5.0,
        thr_min=0.0, thr_max=1.0, thr_cruise=0.5,
        pitch_min=np.radians(-15), pitch_max=np.radians(15),
        airspeed_min=25.0, airspeed_max=60.0, airspeed_cruise=40.0,
    )
    tecs.reset()

    # 每个控制步长调用：
    tecs.update(
        height=state.altitude, climb_rate=hdot,
        airspeed=state.airspeed, accel_body_x=ax,
        roll_rad=state.phi,
        hgt_dem=100.0, airspeed_dem=40.0,
        dt=0.02,
    )
    throttle_cmd = tecs.output.throttle_dem
    pitch_cmd    = tecs.output.pitch_dem
    """

    # ------------------------------------------------------------------
    # 构造 / 参数
    # ------------------------------------------------------------------
    def __init__(
        self,
        # 性能参数（需根据飞机调整）
        max_climb_rate:    float = 5.0,    # TECS_CLMB_MAX  (m/s)
        min_sink_rate:     float = 2.0,    # TECS_SINK_MIN  (m/s)
        max_sink_rate:     float = 5.0,    # TECS_SINK_MAX  (m/s)
        # 控制器调参参数
        time_const:        float = 5.0,    # TECS_TIME_CONST (s)
        thr_damp:          float = 0.5,    # TECS_THR_DAMP
        ptch_damp:         float = 0.3,    # TECS_PTCH_DAMP
        integ_gain:        float = 0.3,    # TECS_INTEG_GAIN
        vert_acc_lim:      float = 7.0,    # TECS_VERT_ACC   (m/s²)
        spd_weight:        float = 1.0,    # TECS_SPDWEIGHT  (0 高度优先 ~ 2 速度优先)
        roll_comp:         float = 10.0,   # TECS_RLL2THR 坡度补偿
        hgt_dem_tconst:    float = 3.0,    # TECS_HDEM_TCONST 高度需求低通 (s)
        # 油门 / 俯仰限制
        thr_min:           float = 0.0,
        thr_max:           float = 1.0,
        thr_cruise:        float = 0.5,
        pitch_min:         float = None,   # rad, None → -np.radians(15)
        pitch_max:         float = None,   # rad, None → +np.radians(15)
        # 空速限制
        airspeed_min:      float = 25.0,   # m/s (EAS ≈ TAS at low alt)
        airspeed_max:      float = 60.0,
        airspeed_cruise:   float = 40.0,
    ):
        self.max_climb_rate  = float(max_climb_rate)
        self.min_sink_rate   = float(min_sink_rate)
        self.max_sink_rate   = float(max_sink_rate)
        self.time_const      = float(time_const)
        self.thr_damp        = float(thr_damp)
        self.ptch_damp       = float(ptch_damp)
        self.integ_gain      = float(integ_gain)
        self.vert_acc_lim    = float(vert_acc_lim)
        self.spd_weight      = float(spd_weight)
        self.roll_comp       = float(roll_comp)
        self.hgt_dem_tconst  = float(hgt_dem_tconst)

        self.thr_min         = float(thr_min)
        self.thr_max         = float(thr_max)
        self.thr_cruise      = float(thr_cruise)
        self.pitch_min       = float(pitch_min) if pitch_min is not None else -np.radians(15.0)
        self.pitch_max       = float(pitch_max) if pitch_max is not None else +np.radians(15.0)

        self.airspeed_min    = float(airspeed_min)
        self.airspeed_max    = float(airspeed_max)
        self.airspeed_cruise = float(airspeed_cruise)

        # 输出
        self.output = TECSState()

        # 内部状态（将在 reset() 中初始化）
        self._reset_needed = True
        self._integTHR   = 0.0   # 油门积分器
        self._integSEBdot = 0.0  # 能量平衡率积分器
        self._integKE    = 0.0   # 动能修正积分器

        self._TAS_state  = airspeed_cruise   # 估计真空速
        self._integDTAS  = 0.0               # 空速互补滤波内部积分

        self._height     = 0.0
        self._climb_rate = 0.0

        # 高度需求滤波器状态
        self._hgt_dem_lpf       = 0.0
        self._hgt_dem_rate_ltd  = 0.0
        self._hgt_dem_in_prev   = 0.0
        self._hgt_dem_prev      = 0.0
        self._hgt_rate_dem      = 0.0
        self._hgt_dem           = 0.0

        # 空速需求
        self._TAS_dem     = airspeed_cruise
        self._TAS_dem_adj = airspeed_cruise
        self._TAS_rate_dem = 0.0
        self._TAS_rate_dem_lpf = 0.0

        # 速率误差上次值（一阶低通）
        self._STEdotErrLast = 0.0

        # 油门 / 俯仰上一步
        self._last_throttle_dem = thr_cruise
        self._last_pitch_dem    = 0.0

        # 前向加速度低通
        self._vel_dot     = 0.0
        self._vel_dot_lpf = 0.0

        # 各种 clip 状态
        self._thr_clip_status  = 0   # -1 min / 0 none / 1 max
        self._SEBdot_dem_clip  = 0

        # pitch_dem 未限幅版本（供 _update_height_demand 中 clip 检查用）
        self._pitch_dem_unc = 0.0

        # 能量相关中间量（供调试记录）
        self._SPE_dem = 0.0; self._SKE_dem = 0.0
        self._SPE_dem_raw = 0.0   # 原始（未低通）目标高度势能，用于油门 STE_error
        self._hgt_dem_raw = 0.0   # 原始目标高度（m）
        self._SPE_est = 0.0; self._SKE_est = 0.0
        self._SPEdot  = 0.0; self._SKEdot  = 0.0
        self._STE_error = 0.0

        self._STEdot_max = max_climb_rate * 9.80665
        self._STEdot_min = -min_sink_rate * 9.80665

        self._underspeed  = False
        self._bad_descent = False

        self._max_climb_scaler = 1.0
        self._max_sink_scaler  = 1.0
        self._sink_fraction    = 0.0

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def reset(self, height: float = 0.0, airspeed: float = None, pitch: float = 0.0):
        """重置所有积分器和滤波器状态。在模式切换或初始化时调用。"""
        if airspeed is None:
            airspeed = self.airspeed_cruise
        self._reset_needed = False

        self._integTHR    = 0.0
        self._integSEBdot = 0.0
        self._integKE     = 0.0

        self._TAS_state  = float(airspeed)
        self._integDTAS  = 0.0

        self._height     = float(height)
        self._climb_rate = 0.0

        self._hgt_dem_lpf       = float(height)
        self._hgt_dem_rate_ltd  = float(height)
        self._hgt_dem_in_prev   = float(height)
        self._hgt_dem_prev      = float(height)
        self._hgt_dem           = float(height)
        self._hgt_dem_raw       = float(height)   # 原始目标高度
        self._hgt_rate_dem      = 0.0

        self._TAS_dem      = float(airspeed)
        self._TAS_dem_adj  = float(airspeed)
        self._TAS_rate_dem = 0.0
        self._TAS_rate_dem_lpf = 0.0

        self._STEdotErrLast    = 0.0
        self._last_throttle_dem = self.thr_cruise
        self._last_pitch_dem    = float(pitch)

        self._vel_dot     = 0.0
        self._vel_dot_lpf = 0.0
        self._thr_clip_status = 0
        self._SEBdot_dem_clip = 0
        self._pitch_dem_unc   = float(pitch)

        self._underspeed  = False
        self._bad_descent = False
        self._max_climb_scaler = 1.0
        self._max_sink_scaler  = 1.0
        self._sink_fraction    = 0.0

        self.output.throttle_dem = self.thr_cruise
        self.output.pitch_dem    = float(pitch)
        self.output.underspeed   = False
        self.output.bad_descent  = False

    def update(
        self,
        height:        float,      # 当前高度 (m, 正向上)
        climb_rate:    float,      # 当前爬升率 (m/s, 正向上)
        airspeed:      float,      # 当前真空速 (m/s)
        accel_body_x:  float,      # 机体 x 轴加速度（沿速度矢量方向, m/s²）
        roll_rad:      float,      # 滚转角 (rad)
        hgt_dem:       float,      # 目标高度 (m)
        airspeed_dem:  float,      # 目标空速 (m/s)
        dt:            float,      # 时间步长 (s)
    ) -> TECSState:
        """
        执行一次 TECS 控制计算。

        Returns
        -------
        TECSState  (同时写入 self.output)
        """
        dt = max(dt, 1e-4)

        # --- 0. 高度 / 爬升率直接使用外部估计 ---------------------
        self._height     = float(height)
        self._climb_rate = float(climb_rate)

        # --- 1. 空速互补滤波 --------------------------------------
        self._update_speed(airspeed, accel_body_x, dt)

        # --- 2. 计算 STE 速率上下限 --------------------------------
        climb_rate_lim = self.max_climb_rate * self._max_climb_scaler
        sink_rate_lim  = self.max_sink_rate  * self._max_sink_scaler
        self._STEdot_max = climb_rate_lim * 9.80665
        self._STEdot_min = -self.min_sink_rate * 9.80665

        # --- 3. 更新空速需求 （带速率限制）------------------------
        self._EAS_dem  = _saturate(float(airspeed_dem), self.airspeed_min, self.airspeed_max)
        self._TAS_dem  = self._EAS_dem          # 低空 EAS ≈ TAS
        self._update_speed_demand(dt)

        # --- 4. 更新高度需求（带速率限制 + 低通）-----------------
        self._hgt_dem_raw = float(hgt_dem)   # 保存原始目标高度（用于油门 STE_error）
        self._update_height_demand(float(hgt_dem), climb_rate_lim, sink_rate_lim, dt)

        # --- 5. 欠速保护 ------------------------------------------
        self._detect_underspeed()

        # --- 6. 更新能量估计与需求 --------------------------------
        self._update_energies()

        # --- 7. 俯仰角指令 ----------------------------------------
        self._update_pitch(roll_rad, dt)

        # --- 8. 油门指令 ------------------------------------------
        self._update_throttle(roll_rad, dt)

        # --- 9. 不可达下沉检测 ------------------------------------
        self._detect_bad_descent()

        # --- 10. 高度 / 爬升率 scaler 自适应 ----------------------
        self._update_climb_sink_scalers(dt)

        # 填充输出
        self.output.throttle_dem = self._throttle_dem
        self.output.pitch_dem    = self._pitch_dem
        self.output.climb_rate   = self._climb_rate
        self.output.height       = self._height
        self.output.airspeed     = self._TAS_state
        self.output.underspeed   = self._underspeed
        self.output.bad_descent  = self._bad_descent
        return self.output

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------

    def _update_speed(self, airspeed_meas: float, accel_body_x: float, dt: float):
        """
        二阶互补滤波估计真空速。
        简化：直接信任传感器，使用一阶低通平滑；加速度用于 vel_dot。
        与 ArduPilot _update_speed 保持结构一致。
        """
        omega = 2.0   # SPD_OMEGA  (rad/s)
        airspeed_meas = max(airspeed_meas, 3.0)

        # 空速互补滤波（二阶）
        asp_err = airspeed_meas - self._TAS_state
        self._integDTAS += asp_err * omega * omega * dt
        TAS_input = (self._integDTAS
                     + accel_body_x           # vel_dot from body accel
                     + asp_err * omega * 1.4142)
        self._TAS_state += TAS_input * dt
        self._TAS_state  = max(self._TAS_state, 3.0)

        # 加速度低通 (vel_dot)
        # ArduPilot 使用固定长时间常数（约20s）来平滑加速度偏置
        # 高通效果：vel_dot - vel_dot_lpf 去除互补滤波引入的低频偏置
        _VEL_DOT_LPF_TC = 20.0   # seconds (ArduPilot AP_TECS default)
        alpha_lpf = dt / (dt + _VEL_DOT_LPF_TC)
        self._vel_dot     = float(accel_body_x)
        self._vel_dot_lpf = self._vel_dot_lpf * (1.0 - alpha_lpf) + self._vel_dot * alpha_lpf

    def _update_speed_demand(self, dt: float):
        """速率限制空速需求，输出 _TAS_dem_adj 和 _TAS_rate_dem。"""
        # 速率限制：用 STEdot 上下限推算允许的速度变化率
        TAS_state_safe = max(self._TAS_state, 3.0)
        vel_rate_max =  0.5 * self._STEdot_max / TAS_state_safe
        vel_rate_min = -0.9 * self.min_sink_rate * 9.80665 / TAS_state_safe

        prev_adj = self._TAS_dem_adj
        delta_dem = self._TAS_dem - prev_adj

        if delta_dem > vel_rate_max * dt:
            self._TAS_dem_adj  = prev_adj + vel_rate_max * dt
            self._TAS_rate_dem = vel_rate_max
        elif delta_dem < vel_rate_min * dt:
            self._TAS_dem_adj  = prev_adj + vel_rate_min * dt
            self._TAS_rate_dem = vel_rate_min
        else:
            self._TAS_dem_adj  = self._TAS_dem
            self._TAS_rate_dem = delta_dem / dt if dt > 0 else 0.0

        # 低通滤波 TAS_rate_dem
        alpha = dt / (dt + self.time_const)
        self._TAS_rate_dem_lpf = (self._TAS_rate_dem_lpf * (1.0 - alpha)
                                  + self._TAS_rate_dem * alpha)
        self._TAS_dem_adj = _saturate(self._TAS_dem_adj, self.airspeed_min, self.airspeed_max)

    def _update_height_demand(
        self,
        hgt_dem_in: float,
        climb_rate_lim: float,
        sink_rate_lim: float,
        dt: float,
    ):
        """
        速率限制 + 一阶低通平滑高度需求。
        输出 _hgt_dem（送入能量计算）和 _hgt_rate_dem（前馈爬升率需求）。
        """
        tc = max(self.hgt_dem_tconst, dt)

        # 2 点滑动平均
        hgt_avg = 0.5 * (hgt_dem_in + self._hgt_dem_in_prev)
        self._hgt_dem_in_prev = hgt_dem_in

        # 速率限制
        delta_hgt = hgt_avg - self._hgt_dem_rate_ltd
        if delta_hgt > climb_rate_lim * dt:
            self._hgt_dem_rate_ltd += climb_rate_lim * dt
            self._sink_fraction = 0.0
        elif delta_hgt < -sink_rate_lim * dt:
            self._hgt_dem_rate_ltd -= sink_rate_lim * dt
            self._sink_fraction = 1.0
        else:
            self._hgt_dem_rate_ltd = hgt_avg
            denom = -sink_rate_lim * dt
            if delta_hgt < 0 and abs(denom) > 1e-6:
                self._sink_fraction = delta_hgt / denom
            else:
                self._sink_fraction = 0.0

        # 一阶低通
        coef = min(dt / (dt + tc), 1.0)
        self._hgt_rate_dem = (self._hgt_dem_rate_ltd - self._hgt_dem_lpf) / tc
        self._hgt_dem_lpf  = (self._hgt_dem_rate_ltd * coef
                               + (1.0 - coef) * self._hgt_dem_lpf)
        self._hgt_dem      = self._hgt_dem_lpf

        # 防止 demand 跑太远（非着陆）
        max_climb_cond  = (self._pitch_dem_unc > self.pitch_max
                           or self._thr_clip_status == 1)
        max_descent_cond = (self._pitch_dem_unc < self.pitch_min
                            or self._thr_clip_status == -1)
        hgt_dem_alpha = dt / max(dt + tc, dt)
        if max_climb_cond and self._hgt_dem > self._hgt_dem_prev:
            self._max_climb_scaler *= (1.0 - hgt_dem_alpha)
        elif max_descent_cond and self._hgt_dem < self._hgt_dem_prev:
            self._max_sink_scaler *= (1.0 - hgt_dem_alpha)
        self._hgt_dem_prev = self._hgt_dem

    def _update_climb_sink_scalers(self, dt: float):
        """恢复 scaler 趋近 1.0。"""
        tc = max(self.hgt_dem_tconst, dt)
        alpha = dt / max(dt + tc, dt)
        self._max_climb_scaler = self._max_climb_scaler * (1.0 - alpha) + alpha
        self._max_sink_scaler  = self._max_sink_scaler  * (1.0 - alpha) + alpha

    def _detect_underspeed(self):
        """欠速保护检测。"""
        TASmin = self.airspeed_min
        if self._TAS_state < TASmin * 0.9 and self._last_throttle_dem >= self.thr_max * 0.95:
            self._underspeed = True
        elif self._underspeed and self._TAS_state >= TASmin * 1.15:
            self._underspeed = False
        elif self._height >= self._hgt_dem and not self._underspeed:
            self._underspeed = False

        if self._underspeed:
            self._TAS_dem_adj = TASmin

    def _update_energies(self):
        """计算比能量需求与估计值。"""
        g = 9.80665
        # 需求
        self._SPE_dem = self._hgt_dem * g          # 低通后的高度需求（用于俯仰）
        self._SPE_dem_raw = self._hgt_dem_raw * g  # 原始目标高度（用于油门 STE_error）
        self._SKE_dem = 0.5 * self._TAS_dem_adj ** 2

        # 估计
        self._SPE_est = self._height * g
        self._SKE_est = 0.5 * self._TAS_state ** 2

        # 速率（高通过滤 vel_dot 以去除互补滤波引入的偏置）
        self._SPEdot = self._climb_rate * g
        self._SKEdot = self._TAS_state * (self._vel_dot - self._vel_dot_lpf)

        # 需求速率（基于原始目标高度，确保爬升响应不被低通削弱）
        self._SPEdot_dem = (self._SPE_dem_raw - self._SPE_est) / self.time_const
        self._SKEdot_dem = self._TAS_state * (self._TAS_rate_dem - self._TAS_rate_dem_lpf)

    def _update_pitch(self, roll_rad: float, dt: float):
        """
        根据比能量分配比 (SEB) 计算俯仰角指令。
        """
        g = 9.80665
        TAS = max(self._TAS_state, 3.0)
        gain_inv = TAS * g  # gainInv = V * g

        # 能量权重（与 ArduPilot AP_TECS._update_pitch 完全一致）
        # spd_weight ∈ [0,2]: 0=纯高度控制, 1=均衡, 2=纯速度控制
        # 先不限幅求两个分量，再各自 cap 到 1.0
        w_ske_raw = _saturate(self.spd_weight, 0.0, 2.0)
        w_spe_raw = 2.0 - w_ske_raw
        w_ske = min(w_ske_raw, 1.0)  # cap at 1
        w_spe = min(w_spe_raw, 1.0)  # cap at 1
        if self._underspeed:
            w_ske = 1.0   # 欠速：速度优先
            w_spe = 1.0

        # SEB 误差
        SEB_dem  = self._SPE_dem * w_spe - self._SKE_dem * w_ske
        SEB_est  = self._SPE_est * w_spe - self._SKE_est * w_ske
        SEB_error = SEB_dem - SEB_est

        # SEBdot 需求 = 前馈 + 比例
        SEBdot_dem = self._hgt_rate_dem * g * w_spe + SEB_error / self.time_const

        # SEBdot 限幅
        SEBdot_max = self.max_climb_rate * g
        SEBdot_min = -self.max_sink_rate * g
        if SEBdot_dem > SEBdot_max:
            SEBdot_dem = SEBdot_max; self._SEBdot_dem_clip = 1
        elif SEBdot_dem < SEBdot_min:
            SEBdot_dem = SEBdot_min; self._SEBdot_dem_clip = -1
        else:
            self._SEBdot_dem_clip = 0

        # SEBdot 估计
        SEBdot_est = self._SPEdot * w_spe - self._SKEdot * w_ske
        SEBdot_error = SEBdot_dem - SEBdot_est

        # 合计需求 = 前馈 + 阻尼
        SEBdot_dem_total = SEBdot_dem + SEBdot_error * self.ptch_damp

        # 积分器限幅（允许 ±5deg 的饱和裕量）
        integ_min = (gain_inv * (self.pitch_min - np.radians(5.0))) - SEBdot_dem_total
        integ_max = (gain_inv * (self.pitch_max + np.radians(5.0))) - SEBdot_dem_total

        # 预测未限幅俯仰
        self._pitch_dem_unc = ((SEBdot_dem_total + self._integSEBdot + self._integKE)
                               / gain_inv)

        # 积分更新（抗饱和）
        integ_range = integ_max - integ_min
        integSEB_delta = _saturate(
            SEBdot_error * self.integ_gain * dt,
            -integ_range * 0.1,
             integ_range * 0.1,
        )
        inhibit = ((self._pitch_dem_unc > self.pitch_max and integSEB_delta > 0)
                   or (self._pitch_dem_unc < self.pitch_min and integSEB_delta < 0))
        if not inhibit:
            self._integSEBdot += integSEB_delta
            self._integKE += ((self._SKE_est - self._SKE_dem) * w_ske * dt
                              / self.time_const)
        else:
            decay = 1.0 - dt / max(dt + self.time_const, dt)
            self._integSEBdot *= decay
            self._integKE *= decay

        self._integSEBdot = _saturate(self._integSEBdot, integ_min, integ_max)
        ke_lim = 0.25 * (self.pitch_max - self.pitch_min) * gain_inv
        self._integKE = _saturate(self._integKE, -ke_lim, ke_lim)

        # 最终俯仰
        self._pitch_dem_unc = ((SEBdot_dem_total + self._integSEBdot + self._integKE)
                               / gain_inv)

        # 限幅并速率限制
        self._pitch_dem = _saturate(self._pitch_dem_unc, self.pitch_min, self.pitch_max)
        rate_lim = dt * self.vert_acc_lim / TAS
        self._pitch_dem = _saturate(
            self._pitch_dem,
            self._last_pitch_dem - rate_lim,
            self._last_pitch_dem + rate_lim,
        )
        self._last_pitch_dem = self._pitch_dem

    def _update_throttle(self, roll_rad: float, dt: float):
        """
        根据总比能量误差计算油门指令。
        """
        if self._underspeed:
            self._throttle_dem = self.thr_max
            self._constrain_throttle()
            return

        # 总比能量误差（使用原始目标高度 SPE_dem_raw，避免低通滤波延迟导致油门欠响应）
        # 双重限幅：
        #  1. SPE_err 限幅防止超速（基于速度包线）
        #  2. 总 STE_error 限幅防止满油门持续时间过长（最多等于 4 倍爬升能力）
        SPE_err_max = max(self._SKE_est - 0.5 * self.airspeed_min ** 2, 0.0)
        SPE_err_min = min(self._SKE_est - 0.5 * self.airspeed_max ** 2, 0.0)

        SPE_err_raw = _saturate(self._SPE_dem_raw - self._SPE_est, SPE_err_min, SPE_err_max)
        STE_error_raw = SPE_err_raw + self._SKE_dem - self._SKE_est

        # 限制 STE_error 不超过 4 倍最大能量需求（防止初始大误差导致长时间满油门）
        STE_limit = 4.0 * self._STEdot_max * self.time_const
        STE_error = _saturate(STE_error_raw, -STE_limit, STE_limit)
        self._STE_error = STE_error

        STEdot_dem = _saturate(
            self._SPEdot_dem + self._SKEdot_dem,
            self._STEdot_min,
            self._STEdot_max,
        )
        STEdot_error = STEdot_dem - (self._SPEdot + self._SKEdot)

        # 低通平滑 STEdot_error
        filt_coef = min(2.0 * dt, 1.0)
        STEdot_error = (filt_coef * STEdot_error
                        + (1.0 - filt_coef) * self._STEdotErrLast)
        self._STEdotErrLast = STEdot_error

        # 增益 K_thr2STE：油门满量程对应的 STEdot 变化量
        K_thr2STE = max(
            (self._STEdot_max - self._STEdot_min) / max(self.thr_max - self.thr_min, 0.01),
            1.0,
        )
        K_STE2Thr = 1.0 / (self.time_const * K_thr2STE)

        # 前馈油门（巡航油门 + 需求 STEdot）
        # 坡度补偿：转弯时诱导阻力增大
        cos_roll_sq = max(np.cos(roll_rad) ** 2, 0.1)
        roll_comp_term = self.roll_comp * (1.0 / cos_roll_sq - 1.0)
        STEdot_ff = STEdot_dem + roll_comp_term
        ff_throttle = self.thr_cruise + STEdot_ff / K_thr2STE

        # PD + FF 油门
        throttle_pd = (STE_error + STEdot_error * self.thr_damp) * K_STE2Thr + ff_throttle

        # 积分器更新（ArduPilot AP_TECS 风格抗饱和逻辑）
        # 当输出饱和且积分方向会加剧饱和时，阻止积分更新
        max_amp  = 0.5 * (self.thr_max - self.thr_min)
        integ_update = STE_error * self.integ_gain * dt * K_STE2Thr

        # 抗饱和：若输出已经饱和且积分方向会加剧饱和，则阻止积分
        saturated_high = (throttle_pd + self._integTHR) >= self.thr_max
        saturated_low  = (throttle_pd + self._integTHR) <= self.thr_min
        if (saturated_high and integ_update > 0) or (saturated_low and integ_update < 0):
            integ_update = 0.0  # 阻止进一步饱和

        self._integTHR += integ_update
        self._integTHR  = _saturate(self._integTHR, -max_amp, max_amp)

        self._throttle_dem = throttle_pd + self._integTHR
        self._last_throttle_dem = self._throttle_dem
        self._constrain_throttle()

    def _constrain_throttle(self):
        if self._throttle_dem > self.thr_max:
            self._thr_clip_status = 1
            self._throttle_dem = self.thr_max
        elif self._throttle_dem < self.thr_min:
            self._thr_clip_status = -1
            self._throttle_dem = self.thr_min
        else:
            self._thr_clip_status = 0

    def _detect_bad_descent(self):
        """检测由于不可达空速需求引起的不可控下沉。"""
        STEdot = self._SPEdot + self._SKEdot
        if (self._STE_error > 200.0
                and STEdot < 0.0
                and self._last_throttle_dem >= self.thr_max * 0.9):
            self._bad_descent = True
        elif self._bad_descent and self._STE_error > 0.0:
            pass  # 保持激活直到 STE_error 降到 0
        else:
            self._bad_descent = False
        self.output.bad_descent = self._bad_descent
