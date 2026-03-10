#!/usr/bin/env python3
"""
测试仿真器的 PX4 和 ArduPilot 控制器切换功能（方案A：独立调参）
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from simulation.simulator import FixedWingSimulator


def test_ardupilot_controller():
    """测试 ArduPilot 控制器"""
    print("=" * 60)
    print("测试 ArduPilot 控制器")
    print("=" * 60)
    
    sim = FixedWingSimulator(
        aircraft_name="TB2",
        dt=0.02,
        duration=5.0,
        controller_type="ardupilot"
    )
    
    print(f"✓ 控制器类型: {sim.controller_type}")
    print(f"✓ 姿态控制器: {type(sim.att_ctrl).__name__}")
    print(f"✓ 速率控制器: {type(sim.rate_ctrl).__name__}")
    print()
    
    # 测试 ArduPilot 参数设置
    sim.set_ardupilot_params(
        roll_attitude={'p': 1.2},
        roll_rate={'p': 0.08, 'i': 0.12, 'd': 0.003, 'ff': 0.20}
    )
    print("✓ ArduPilot 参数设置成功")
    print()
    
    # 测试错误：尝试设置 PX4 参数
    print("测试错误处理：尝试在 ArduPilot 控制器上设置 PX4 参数...")
    try:
        sim.set_px4_params(roll_attitude={'tc': 0.5})
        print("✗ 应该抛出异常但没有")
        return False
    except RuntimeError as e:
        print(f"✓ 正确抛出异常: {e}")
        print()
    
    return True


def test_px4_controller():
    """测试 PX4 控制器"""
    print("=" * 60)
    print("测试 PX4 控制器")
    print("=" * 60)
    
    sim = FixedWingSimulator(
        aircraft_name="TB2",
        dt=0.02,
        duration=5.0,
        controller_type="px4"
    )
    
    print(f"✓ 控制器类型: {sim.controller_type}")
    print(f"✓ 姿态控制器: {type(sim.att_ctrl).__name__}")
    print(f"✓ 速率控制器: {type(sim.rate_ctrl).__name__}")
    print()
    
    # 测试 PX4 参数设置
    sim.set_px4_params(
        roll_attitude={'tc': 0.4},
        roll_rate={'kp': 0.08, 'ki': 0.12, 'kff': 0.45}
    )
    print("✓ PX4 参数设置成功")
    print()
    
    # 测试错误：尝试设置 ArduPilot 参数
    print("测试错误处理：尝试在 PX4 控制器上设置 ArduPilot 参数...")
    try:
        sim.set_ardupilot_params(roll_attitude={'p': 1.0})
        print("✗ 应该抛出异常但没有")
        return False
    except RuntimeError as e:
        print(f"✓ 正确抛出异常: {e}")
        print()
    
    return True


def test_config_file_switch():
    """测试从配置文件读取控制器类型"""
    print("=" * 60)
    print("测试从配置文件读取控制器类型")
    print("=" * 60)
    
    # 默认从 config/simulation.yaml 读取
    sim = FixedWingSimulator(
        aircraft_name="TB2",
        dt=0.02,
        duration=5.0
    )
    
    print(f"✓ 从配置文件读取的控制器类型: {sim.controller_type}")
    print(f"✓ 姿态控制器: {type(sim.att_ctrl).__name__}")
    print(f"✓ 速率控制器: {type(sim.rate_ctrl).__name__}")
    print()
    
    return True


if __name__ == "__main__":
    try:
        success = True
        success = test_ardupilot_controller() and success
        success = test_px4_controller() and success
        success = test_config_file_switch() and success
        
        if success:
            print("=" * 60)
            print("✓ 所有测试通过！")
            print("=" * 60)
            print()
            print("方案 A：独立调参")
            print("-" * 60)
            print("每种控制律使用独立的最优参数，不进行参数映射。")
            print()
            print("使用说明：")
            print("1. 创建 ArduPilot 控制器：")
            print("   sim = FixedWingSimulator(controller_type='ardupilot')")
            print("   sim.set_ardupilot_params(...)")
            print()
            print("2. 创建 PX4 控制器：")
            print("   sim = FixedWingSimulator(controller_type='px4')")
            print("   sim.set_px4_params(...)")
            print()
            print("3. 从配置文件读取：")
            print("   在 config/simulation.yaml 中设置 controller_type")
            print()
            print("注意：")
            print("- 不支持参数映射，必须使用对应控制器的参数格式")
            print("- 每种控制律独立调参，达到最优性能")
            print("- 切换控制器需要重新创建仿真器实例")
        else:
            print("✗ 部分测试失败")
            sys.exit(1)
        
    except Exception as e:
        print(f"✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
