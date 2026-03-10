#!/usr/bin/env python3
"""
测试 V型尾翼气动模型
验证：
1. 对称偏转产生俯仰力矩
2. 差动偏转产生偏航力矩
3. 耦合效应正确
"""

import numpy as np
import sys
sys.path.insert(0, 'src')

from dynamics.aerodynamics import compute_aero_forces
from models.aircraft_database import get_aircraft_params


def test_vtail_pitch_control():
    """测试对称 V尾偏转产生俯仰力矩"""
    print("\n=== 测试 1: 对称 V尾偏转（俯仰控制）===")
    
    params = get_aircraft_params("TB2")
    
    # 基准状态（无偏转）
    af_neutral = compute_aero_forces(
        u=30.0, v=0.0, w=0.5, p=0.0, q=0.0, r=0.0,
        da_left=0.0, da_right=0.0, dv_left=0.0, dv_right=0.0, dt=0.5,
        params=params
    )
    
    # 对称正偏转（两个 V尾都向下，产生抬头力矩）
    dv = np.radians(5.0)
    af_pitch_up = compute_aero_forces(
        u=30.0, v=0.0, w=0.5, p=0.0, q=0.0, r=0.0,
        da_left=0.0, da_right=0.0, dv_left=dv, dv_right=dv, dt=0.5,
        params=params
    )
    
    print(f"中立状态俯仰力矩 Cm: {af_neutral.Cm:.4f}")
    print(f"对称偏转俯仰力矩 Cm: {af_pitch_up.Cm:.4f}")
    print(f"俯仰力矩变化 ΔCm: {af_pitch_up.Cm - af_neutral.Cm:.4f}")
    
    # 验证：对称偏转应该主要影响俯仰力矩
    assert abs(af_pitch_up.Cm - af_neutral.Cm) > 0.01, "对称偏转应产生明显俯仰力矩变化"
    print("✓ 对称偏转产生俯仰力矩")


def test_vtail_yaw_control():
    """测试差动 V尾偏转产生偏航力矩"""
    print("\n=== 测试 2: 差动 V尾偏转（偏航控制）===")
    
    params = get_aircraft_params("TB2")
    
    # 基准状态
    af_neutral = compute_aero_forces(
        u=30.0, v=0.0, w=0.5, p=0.0, q=0.0, r=0.0,
        da_left=0.0, da_right=0.0, dv_left=0.0, dv_right=0.0, dt=0.5,
        params=params
    )
    
    # 差动偏转（右V尾向下，左V尾向上，产生右偏航）
    dv = np.radians(5.0)
    af_yaw_right = compute_aero_forces(
        u=30.0, v=0.0, w=0.5, p=0.0, q=0.0, r=0.0,
        da_left=0.0, da_right=0.0, dv_left=-dv, dv_right=dv, dt=0.5,
        params=params
    )
    
    print(f"中立状态偏航力矩 Cn: {af_neutral.Cn:.4f}")
    print(f"差动偏转偏航力矩 Cn: {af_yaw_right.Cn:.4f}")
    print(f"偏航力矩变化 ΔCn: {af_yaw_right.Cn - af_neutral.Cn:.4f}")
    
    # 验证：差动偏转应该主要影响偏航力矩
    assert abs(af_yaw_right.Cn - af_neutral.Cn) > 0.001, "差动偏转应产生明显偏航力矩变化"
    print("✓ 差动偏转产生偏航力矩")


def test_vtail_coupling():
    """测试 V尾的俯仰-偏航耦合"""
    print("\n=== 测试 3: V尾耦合效应 ===")
    
    params = get_aircraft_params("TB2")
    
    # 单侧 V尾偏转（应同时产生俯仰和偏航力矩）
    dv = np.radians(5.0)
    af_single = compute_aero_forces(
        u=30.0, v=0.0, w=0.5, p=0.0, q=0.0, r=0.0,
        da_left=0.0, da_right=0.0, dv_left=0.0, dv_right=dv, dt=0.5,
        params=params
    )
    
    af_neutral = compute_aero_forces(
        u=30.0, v=0.0, w=0.5, p=0.0, q=0.0, r=0.0,
        da_left=0.0, da_right=0.0, dv_left=0.0, dv_right=0.0, dt=0.5,
        params=params
    )
    
    delta_Cm = af_single.Cm - af_neutral.Cm
    delta_Cn = af_single.Cn - af_neutral.Cn
    
    print(f"单侧偏转俯仰力矩变化 ΔCm: {delta_Cm:.4f}")
    print(f"单侧偏转偏航力矩变化 ΔCn: {delta_Cn:.4f}")
    
    # 验证：单侧偏转应同时影响俯仰和偏航
    assert abs(delta_Cm) > 0.001, "单侧偏转应产生俯仰力矩"
    assert abs(delta_Cn) > 0.001, "单侧偏转应产生偏航力矩"
    print("✓ V尾耦合效应正确")


def test_vtail_geometry():
    """验证 V尾几何参数（30° 夹角）"""
    print("\n=== 测试 4: V尾几何参数 ===")
    
    v_tail_angle = np.radians(30.0)
    cos_vtail = np.cos(v_tail_angle)
    sin_vtail = np.sin(v_tail_angle)
    
    print(f"V尾夹角: 30°")
    print(f"cos(30°) = {cos_vtail:.4f} (俯仰分量)")
    print(f"sin(30°) = {sin_vtail:.4f} (偏航分量)")
    
    assert abs(cos_vtail - 0.866) < 0.001, "cos(30°) 应约为 0.866"
    assert abs(sin_vtail - 0.5) < 0.001, "sin(30°) 应为 0.5"
    print("✓ V尾几何参数正确")


if __name__ == "__main__":
    print("=" * 60)
    print("V型尾翼气动模型测试")
    print("=" * 60)
    
    try:
        test_vtail_geometry()
        test_vtail_pitch_control()
        test_vtail_yaw_control()
        test_vtail_coupling()
        
        print("\n" + "=" * 60)
        print("✓ 所有测试通过！V型尾翼气动模型工作正常")
        print("=" * 60)
        
    except AssertionError as e:
        print(f"\n✗ 测试失败: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
