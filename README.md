# FixedWingSimulator

专业级固定翼 UAV 仿真与控制平台，融合 ArduPilot Plane 控制架构与 Minimum Snap 轨迹规划，支持 7 种真实飞行器参数。

---

## 目录结构

```
FixedWingSimulator/
├── config/                          # YAML 配置文件
│   ├── aircraft.yaml                # 飞行器选择与参数覆盖
│   ├── control_params.yaml          # ArduPilot 格式控制参数
│   ├── simulation.yaml              # 仿真参数（dt、时长等）
│   └── trajectory.yaml              # 轨迹航点配置
├── src/                             # 核心源码
│   ├── dynamics/                    # 飞行动力学模型
│   ├── control/                     # ArduPilot 5层控制系统
│   ├── planning/                    # 轨迹规划（Minimum Snap/Jerk）
│   ├── environment/                 # 大气模型 & 风场模型
│   ├── models/                      # 飞行器参数数据库（7种UAV）
│   ├── simulation/                  # 仿真引擎 & 积分器 & 状态管理
│   ├── visualization/               # 可视化（Plotly/Matplotlib/动画）
│   └── utils/                       # 数学工具 & 配置加载 & 日志
├── examples/                        # 7 个完整示例脚本
├── tests/                           # 单元测试 & 集成测试（121个）
├── main.py                          # 命令行入口
├── requirements.txt                 # 依赖列表
└── setup.py                         # 包安装配置
```

---

## 环境配置

**要求：** Python 3.8+

### 创建虚拟环境并安装依赖

```bash
cd FixedWingSimulator

# 创建虚拟环境
python3 -m venv .venv

# 激活虚拟环境
source .venv/bin/activate          # Linux / macOS
# .venv\Scripts\activate           # Windows

# 安装依赖
pip install -r requirements.txt
```

---

## 运行测试

```bash
# 激活虚拟环境后，在 FixedWingSimulator/ 目录下执行：
source .venv/bin/activate

# 运行全部测试（121个）
python -m pytest tests/ -v

# 只运行某一类测试
python -m pytest tests/test_dynamics.py -v    # 动力学模型
python -m pytest tests/test_control.py  -v    # 控制系统
python -m pytest tests/test_planning.py -v    # 轨迹规划
python -m pytest tests/test_integration.py -v # 集成测试

# 快速验证（不显示详情）
python -m pytest tests/ -q
```

---

## 命令行运行

```bash
source .venv/bin/activate
cd FixedWingSimulator

# 默认：TB2，AUTO 模式，30s，Minimum Snap 轨迹
python main.py

# 指定飞行器和飞行模式
python main.py --aircraft Predator --mode FBW_B --duration 60

# 加入风扰动
python main.py --aircraft Anka --mode STABILIZE --wind SINE --duration 30

# 4-DOF 线性分析（模态分析，不运行仿真）
python main.py --aircraft TB2 --analysis 4dof

# 6-DOF 开环仿真
python main.py --aircraft Heron_MK1 --analysis 6dof

# 列出所有可用飞行器
python main.py --list-aircraft

# 不显示图形（批量运行时）
python main.py --aircraft TB2 --duration 60 --no-plot
```

### 命令行参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--aircraft` | `TB2` | 飞行器名称 |
| `--mode` | `AUTO` | 飞行模式 |
| `--duration` | `30.0` | 仿真时长（秒） |
| `--dt` | `0.01` | 仿真步长（秒） |
| `--wind` | `NONE` | 风场模型 |
| `--traj` | `minimum_snap` | 轨迹类型 |
| `--analysis` | `None` | 开环分析模式 |
| `--no-plot` | `False` | 关闭可视化 |
| `--config-dir` | `./config` | 配置文件目录 |

---

## 示例脚本

```bash
source .venv/bin/activate
cd FixedWingSimulator

# 示例 1：4-DOF 线性模态分析（短周期 / 长周期）
python examples/1_linear_response.py

# 示例 2：6-DOF 非线性自由飞行，副翼脉冲响应
python examples/2_nonlinear_dynamics.py

# 示例 3：AUTO 模式 + Minimum Snap 500m 方形轨迹跟踪
python examples/3_trajectory_tracking.py

# 示例 4：四边形环绕飞行，L1 导航控制器测试
python examples/4_circuit_flight.py

# 示例 5：7 种飞行器短周期模态对比
python examples/5_different_aircraft.py

# 示例 6：ArduPilot 参数加载、PTCH_P 增益敏感性分析、导出 .param 文件
python examples/6_ardupilot_parameters.py

# 示例 7：FBW_B 模式 + 随机阵风扰动
python examples/7_wind_resistance.py
```

---

## 支持的飞行器

| 飞行器 | 类型 | 巡航速度 |
|--------|------|----------|
| TB2 | 中型侦察 MALE UAV | ~33 m/s |
| Anka | 中型 MALE UAV | ~35 m/s |
| Aksungur | 重型 MALE UAV | ~37 m/s |
| Karayel | 中型战术 UAV | ~30 m/s |
| Predator | 经典 MALE UAV | ~30 m/s |
| Heron MK1 | 中型侦察 UAV | ~35 m/s |
| Heron MK2 | 升级型 Heron | ~38 m/s |

---

## 飞行模式

| 模式 | 说明 |
|------|------|
| `MANUAL` | 直接舵面控制，无辅助 |
| `STABILIZE` | 姿态增稳，保持水平 |
| `FBW_A` | Fly-By-Wire A：限制姿态角范围 |
| `FBW_B` | Fly-By-Wire B：空速 + 高度保持 |
| `AUTO` | 全自动：轨迹规划 + 航点跟踪 |
| `LOITER` | 定点盘旋 |
| `RTH` | 返航 |

---

## 控制系统（ArduPilot 兼容）

控制参数位于 `config/control_params.yaml`，命名与 ArduPilot Plane 完全一致：

```yaml
# 姿态外环
PTCH_P: 1.0          # 俯仰角比例增益
ROLL_P: 1.0          # 滚转角比例增益
YAW_P: 0.5           # 偏航角比例增益

# 角速率内环
PTCH_RATE_P: 0.04
PTCH_RATE_I: 0.04
PTCH_RATE_D: 0.002
ROLL_RATE_P: 0.15
ROLL_RATE_I: 0.15

# 飞行包线限制
LIM_PITCH_MAX: 20.0  # 最大俯仰角（度）
LIM_ROLL_CD: 4500    # 最大滚转角（厘度）

# L1 导航律
NAVL1_PERIOD: 20.0
NAVL1_DAMPING: 0.75
```

---

## 在代码中调用

```python
import sys
sys.path.insert(0, "src")

from simulation.simulator import FixedWingSimulator

# 创建仿真器
sim = FixedWingSimulator(
    aircraft_name="TB2",
    dt=0.01,
    duration=30.0,
    initial_mode="AUTO",
    wind_type="SINE",
)

# 闭环仿真
result = sim.run(closed_loop=True)
print(result.summary())
result.visualize()

# 4-DOF 线性分析
linear_result = sim.run_linear_analysis()
print(linear_result.summary())

# 逐步仿真（供 UI 实时调用）
state = sim.init_step()
for _ in range(100):
    state = sim.step(dt=0.01)
    print(f"alt={state.altitude:.1f} m, airspeed={state.airspeed:.1f} m/s")
```

---

## 依赖

| 包 | 版本 | 用途 |
|----|------|------|
| numpy | >=1.24 | 数值计算 |
| scipy | >=1.9 | ODE 积分器、优化 |
| matplotlib | >=3.5 | 静态绘图、动画 |
| plotly | >=5.10 | 交互式图表 |
| pyyaml | >=6.0 | 配置文件解析 |
| pandas | >=1.5 | 数据导出（CSV） |
| pytest | >=7.2 | 测试框架 |
