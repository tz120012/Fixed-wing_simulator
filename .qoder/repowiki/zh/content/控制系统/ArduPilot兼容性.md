# ArduPilot兼容性

<cite>
**本文档引用的文件**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py)
- [example_6_ardupilot_parameters.py](file://examples/example_6_ardupilot_parameters.py)
- [aircraft_factory.py](file://src/models/aircraft_factory.py)
- [aircraft_database.py](file://src/models/aircraft_database.py)
- [simulator.py](file://src/simulation/simulator.py)
- [attitude_controller.py](file://src/control/attitude_controller.py)
- [rate_controller.py](file://src/control/rate_controller.py)
- [servo_mixer.py](file://src/control/servo_mixer.py)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py)
- [navigation_controller.py](file://src/control/navigation_controller.py)
- [tecs_controller.py](file://src/control/tecs_controller.py)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py)
- [config_loader.py](file://src/utils/config_loader.py)
- [aircraft.yaml](file://config/aircraft.yaml)
- [control_params.yaml](file://config/control_params.yaml)
</cite>

## 目录
1. [引言](#引言)
2. [项目结构](#项目结构)
3. [核心组件](#核心组件)
4. [架构概览](#架构概览)
5. [详细组件分析](#详细组件分析)
6. [依赖分析](#依赖分析)
7. [性能考虑](#性能考虑)
8. [故障排除指南](#故障排除指南)
9. [结论](#结论)
10. [附录](#附录)

## 引言
本文件面向ArduPilot兼容性模块，系统性阐述FixedWingSimulator与ArduPilot系统的兼容性设计与实现细节。文档聚焦于以下目标：
- 解释ArduPilot参数容器与参数命名规范的映射关系
- 详述控制链路（导航、TECS、姿态、角速率、执行器）如何适配ArduPilot参数
- 说明参数导入/导出机制与双向同步策略
- 提供参数验证、热重载与兼容性测试方法
- 给出常见问题定位与解决方案

## 项目结构
FixedWingSimulator采用模块化分层架构，控制链路严格遵循ArduPilot五层控制层级（导航/L1→TECS→姿态→角速率→执行器），并通过ArduPilot参数容器统一参数来源与命名。

```mermaid
graph TB
subgraph "配置层"
CFG1["aircraft.yaml"]
CFG2["control_params.yaml"]
CFG3["simulation.yaml"]
CFG4["trajectory.yaml"]
end
subgraph "模型层"
DB["aircraft_database.py"]
AF["aircraft_factory.py"]
end
subgraph "仿真引擎"
SIM["simulator.py"]
FM["flight_mode_manager.py"]
NAV["navigation_controller.py"]
ATT["attitude_controller.py"]
RATE["rate_controller.py"]
SERVO["servo_mixer.py"]
WPM["waypoint_manager.py"]
CFG["config_loader.py"]
end
subgraph "控制参数"
APC["ardupilot_compat.py"]
end
CFG1 --> AF
CFG2 --> APC
APC --> SIM
AF --> SIM
DB --> AF
CFG3 --> SIM
CFG4 --> WPM
SIM --> NAV
SIM --> ATT
SIM --> RATE
SIM --> SERVO
SIM --> FM
SIM --> WPM
NAV --> SIM
ATT --> SIM
RATE --> SIM
SERVO --> SIM
FM --> SIM
WPM --> SIM
```

**图表来源**
- [simulator.py](file://src/simulation/simulator.py#L115-L234)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L39-L93)
- [aircraft_database.py](file://src/models/aircraft_database.py#L143-L166)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L99)
- [config_loader.py](file://src/utils/config_loader.py#L59-L82)

**章节来源**
- [simulator.py](file://src/simulation/simulator.py#L115-L234)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L39-L93)
- [aircraft_database.py](file://src/models/aircraft_database.py#L143-L166)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L99)
- [config_loader.py](file://src/utils/config_loader.py#L59-L82)

## 核心组件
- ArduPilot参数容器：提供与ArduPilot Plane完全一致的参数命名与默认值，支持YAML导入/导出与基础范围校验。
- 五层控制链路：导航控制器（L1+TECS）、姿态控制器、角速率控制器、执行器混合器，完整复刻ArduPilot控制层级。
- 飞行模式管理：支持MANUAL、STABILIZE、FBW_A、FBW_B、AUTO、LOITER、RTH等模式，与ArduPilot一致。
- 参数导入/导出：支持从ArduPilot .param文件导入，以及将仿真参数导出为ArduPilot .param格式。
- 热重载与验证：支持运行时热更新参数并进行范围校验，保障稳定性。

**章节来源**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L130)
- [attitude_controller.py](file://src/control/attitude_controller.py#L33-L134)
- [rate_controller.py](file://src/control/rate_controller.py#L32-L109)
- [servo_mixer.py](file://src/control/servo_mixer.py#L54-L153)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [tecs_controller.py](file://src/control/tecs_controller.py#L50-L647)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L94-L136)

## 架构概览
ArduPilot兼容性架构以ArduPilot参数容器为核心，贯穿仿真引擎各模块。控制链路自外向内依次为：导航控制器（L1+TECS）→姿态控制器→角速率控制器→执行器混合器。飞行模式管理器根据当前模式生成控制目标，驱动控制链路完成闭环控制。

```mermaid
sequenceDiagram
participant User as "用户"
participant Sim as "FixedWingSimulator"
participant Nav as "NavigationController"
participant Att as "AttitudeController"
participant Rate as "RateController"
participant Servo as "ServoMixer"
participant Dyn as "NonlinearModel"
User->>Sim : 设置初始模式/任务
Sim->>Nav : update(state, segment, dt)
Nav->>Nav : L1横向导航 + TECS纵向控制
Nav-->>Sim : ControlTarget(roll,pitch,yaw,throttle)
Sim->>Att : update(phi,theta,psi,roll_cmd,pitch_cmd,yaw_cmd,dt)
Att-->>Sim : 角速率命令
Sim->>Rate : update(p,q,r,p_cmd,q_cmd,r_cmd,dt)
Rate-->>Sim : 表面增量
Sim->>Servo : update(elev_in,ail_in,rud_in,throttle,phi,p,dt)
Servo-->>Sim : 最终控制输出(de,da,dr,throttle)
Sim->>Dyn : state_dot(t,y,controls,wind,rho)
Dyn-->>Sim : 状态导数
Sim-->>User : 记录历史/可视化
```

**图表来源**
- [simulator.py](file://src/simulation/simulator.py#L419-L567)
- [navigation_controller.py](file://src/control/navigation_controller.py#L134-L202)
- [attitude_controller.py](file://src/control/attitude_controller.py#L81-L122)
- [rate_controller.py](file://src/control/rate_controller.py#L68-L98)
- [servo_mixer.py](file://src/control/servo_mixer.py#L80-L149)

## 详细组件分析

### ArduPilot参数容器与参数映射
- 参数命名与ArduPilot Plane完全一致，覆盖横滚/俯仰/偏航控制回路、限幅、导航L1、空速/高度设定、TECS参数等。
- 支持从YAML加载（扁平键值）、导出为YAML，以及基础范围校验。
- 提供便捷属性（如LIM_ROLL_DEG）将内部单位转换为仿真使用单位。

```mermaid
classDiagram
class ArdupilotParams {
+PTCH_P : float
+PTCH_RATE_P : float
+PTCH_RATE_I : float
+PTCH_RATE_D : float
+PTCH_RATE_FF : float
+ROLL_P : float
+ROLL_RATE_P : float
+ROLL_RATE_I : float
+ROLL_RATE_D : float
+ROLL_RATE_FF : float
+YAW_RATE_P : float
+YAW_RATE_I : float
+YAW_RATE_D : float
+YAW_RATE_FF : float
+LIM_PITCH_MAX : float
+LIM_PITCH_MIN : float
+LIM_ROLL_CD : float
+THR_MAX : float
+THR_MIN : float
+NAVL1_PERIOD : float
+NAVL1_DAMPING : float
+AIRSPEED_CRUISE : float
+ALT_HOLD_RTL : float
+LIM_ROLL_DEG() float
+from_dict(dict) ArdupilotParams
+from_yaml(path) ArdupilotParams
+to_dict() dict
+to_yaml(path) void
+validate() bool
}
```

**图表来源**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L130)

**章节来源**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L130)
- [control_params.yaml](file://config/control_params.yaml#L1-L45)

### 导航控制器与L1+TECS
- L1横向导航：基于ArduPilot L1算法，结合地面速度投影确定期望航迹，计算横向加速度并转换为横滚角命令。
- TECS纵向控制：总比能量控制，协调油门与俯仰控制高度与空速，具备欠速保护、不可达下沉检测与自适应缩放。
- 导航控制器接收路径段与当前状态，输出ControlTarget（roll_cmd, pitch_cmd, throttle_cmd等）。

```mermaid
flowchart TD
Start(["进入 update"]) --> L1["计算L1横向导航<br/>确定look-ahead点与eta"]
L1 --> RollCmd["计算横向加速度与roll_cmd"]
RollCmd --> TECS["TECS更新<br/>高度/爬升率/空速估计"]
TECS --> PitchThr["计算pitch_cmd与throttle_cmd"]
PitchThr --> Target["封装ControlTarget"]
Target --> End(["返回目标"])
```

**图表来源**
- [navigation_controller.py](file://src/control/navigation_controller.py#L134-L202)
- [tecs_controller.py](file://src/control/tecs_controller.py#L247-L315)

**章节来源**
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [tecs_controller.py](file://src/control/tecs_controller.py#L50-L647)

### 姿态控制器与角速率控制器
- 姿态控制器：独立的三轴PID（Roll P仅、Pitch P仅、Yaw无外环），输出角速率命令。
- 角速率控制器：独立的三轴PID（含P/I/D），作为SAS内环，提供阻尼与稳定。

```mermaid
classDiagram
class AttitudeController {
+update(phi,theta,psi,roll_cmd,pitch_cmd,yaw_cmd,dt) AttitudeOutput
+reload_gains(ap_params) void
+reset() void
}
class RateController {
+update(p,q,r,p_cmd,q_cmd,r_cmd,dt) RateOutput
+reload_gains(ap_params) void
+reset() void
}
class ArdupilotParams
AttitudeController --> ArdupilotParams : "使用PTCH_P/ROLL_P"
RateController --> ArdupilotParams : "使用PTCH_RATE_* / ROLL_RATE_* / YAW_RATE_*"
```

**图表来源**
- [attitude_controller.py](file://src/control/attitude_controller.py#L33-L134)
- [rate_controller.py](file://src/control/rate_controller.py#L32-L109)

**章节来源**
- [attitude_controller.py](file://src/control/attitude_controller.py#L33-L134)
- [rate_controller.py](file://src/control/rate_controller.py#L32-L109)

### 执行器混合器与限幅
- 将表面增量转换为最终控制输出，应用限幅（俯仰/横滚/油门）、速率限制、协调转弯补偿，并输出归一化控制信号。

```mermaid
flowchart TD
In(["输入: elev_in, ail_in, rud_in, throttle, φ, p"]) --> Elev["俯仰限幅与饱和"]
Elev --> Ail["横滚限幅与饱和"]
Ail --> RudCoord["协调转弯rudder补偿"]
RudCoord --> Thr["油门限幅"]
Thr --> RateLimit["速率限制(近似)"]
RateLimit --> Out(["输出: ServoOutput(de, da, dr, throttle)"])
```

**图表来源**
- [servo_mixer.py](file://src/control/servo_mixer.py#L80-L149)

**章节来源**
- [servo_mixer.py](file://src/control/servo_mixer.py#L54-L153)

### 飞行模式管理
- 支持MANUAL、STABILIZE、FBW_A、FBW_B、AUTO、LOITER、RTH等模式，模式切换时进行过渡处理，生成ControlTarget供控制链路使用。

```mermaid
stateDiagram-v2
[*] --> AUTO
AUTO --> LOITER : "进入LOITER"
LOITER --> AUTO : "退出LOITER"
AUTO --> RTH : "进入RTH"
RTH --> AUTO : "到达HOME"
AUTO --> STABILIZE : "切换到STABILIZE"
STABILIZE --> AUTO : "切换回AUTO"
AUTO --> FBW_B : "切换到FBW_B"
FBW_B --> AUTO : "切换回AUTO"
AUTO --> MANUAL : "切换到MANUAL"
MANUAL --> AUTO : "切换回AUTO"
```

**图表来源**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)

**章节来源**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)

### 参数导入/导出与双向同步
- 导入：ArduPilotParams.from_yaml()从YAML加载参数；AircraftFactory.export_ardupilot_params()将飞机参数与控制参数打包为ArduPilot .param文件。
- 导出：ArduPilotParams.to_yaml()导出参数；示例脚本展示热重载与对比实验。
- 双向同步：仿真运行时可通过修改ap_params并调用控制器reload_gains()实现热重载；同时保留ArduPilot参数命名与范围约束。

```mermaid
sequenceDiagram
participant User as "用户"
participant Example as "示例脚本"
participant APC as "ArduPilotParams"
participant AF as "AircraftFactory"
participant Sim as "FixedWingSimulator"
User->>Example : 运行示例
Example->>APC : from_yaml(path)
APC-->>Example : ArdupilotParams实例
Example->>AF : export_ardupilot_params(name, out_path, ctrl_yaml)
AF-->>Example : 导出.param文件
Example->>Sim : 修改ap_params并reload_gains()
Sim-->>Example : 热重载生效
```

**图表来源**
- [example_6_ardupilot_parameters.py](file://examples/example_6_ardupilot_parameters.py#L20-L84)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L94-L136)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L74-L99)

**章节来源**
- [example_6_ardupilot_parameters.py](file://examples/example_6_ardupilot_parameters.py#L1-L85)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L94-L136)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L74-L99)

## 依赖分析
- 模块耦合：仿真引擎通过ArduPilot参数容器统一控制参数来源；控制链路之间存在清晰的输入/输出契约。
- 外部依赖：YAML解析、数值计算库（numpy）、数据类（dataclasses）。
- 参数依赖：导航控制器依赖L1与TECS参数；姿态/角速率控制器依赖对应PID参数；执行器混合器依赖限幅与速率参数。

```mermaid
graph TB
SIM["simulator.py"] --> APC["ardupilot_compat.py"]
SIM --> NAV["navigation_controller.py"]
SIM --> ATT["attitude_controller.py"]
SIM --> RATE["rate_controller.py"]
SIM --> SERVO["servo_mixer.py"]
SIM --> FM["flight_mode_manager.py"]
SIM --> WPM["waypoint_manager.py"]
NAV --> TECS["tecs_controller.py"]
AF["aircraft_factory.py"] --> DB["aircraft_database.py"]
AF --> APC
```

**图表来源**
- [simulator.py](file://src/simulation/simulator.py#L33-L52)
- [navigation_controller.py](file://src/control/navigation_controller.py#L20-L22)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L12-L12)

**章节来源**
- [simulator.py](file://src/simulation/simulator.py#L33-L52)
- [navigation_controller.py](file://src/control/navigation_controller.py#L20-L22)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L12-L12)

## 性能考虑
- 控制链路采样：默认dt=0.01s，保证控制响应与数值积分稳定性。
- 速率限制：执行器混合器对表面增量施加速率限制，避免瞬态过冲。
- TECS自适应：根据爬升/下沉需求动态缩放，提升鲁棒性。
- 热重载：支持运行时参数热更新，便于在线调参与快速验证。

[本节为通用指导，无需列出具体文件来源]

## 故障排除指南
- 参数越界：使用ArduPilotParams.validate()进行范围检查，出现警告时调整参数至合理区间。
- 控制不稳定：检查L1周期与阻尼、TECS时间常数与阻尼系数，逐步调整以改善响应。
- 油门饱和/积分饱和：确认TECS积分增益与油门限幅设置，必要时降低积分增益或增加阻尼。
- 模式切换异常：检查FlightModeManager模式切换逻辑与过渡处理，确保切换时正确重置控制器积分器。

**章节来源**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L104-L130)
- [tecs_controller.py](file://src/control/tecs_controller.py#L507-L534)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L152-L168)

## 结论
ArduPilot兼容性模块通过标准化参数容器与严格的控制链路实现，成功将ArduPilot的控制理念与参数体系移植到仿真环境中。模块支持参数导入/导出、热重载与范围校验，满足从参数迁移、在线调参到闭环仿真的完整需求。建议在实际工程中结合示例脚本与参数验证流程，建立规范化的参数管理与测试流程。

[本节为总结性内容，无需列出具体文件来源]

## 附录

### 支持的ArduPilot参数类型与映射
- 控制回路参数
  - 横滚：ROLL_P、ROLL_RATE_P、ROLL_RATE_I、ROLL_RATE_D、ROLL_RATE_FF
  - 俯仰：PTCH_P、PTCH_RATE_P、PTCH_RATE_I、PTCH_RATE_D、PTCH_RATE_FF
  - 偏航：YAW_RATE_P、YAW_RATE_I、YAW_RATE_D、YAW_RATE_FF
- 限幅参数：LIM_PITCH_MAX、LIM_PITCH_MIN、LIM_ROLL_CD、THR_MAX、THR_MIN
- 导航参数：NAVL1_PERIOD、NAVL1_DAMPING、AIRSPEED_CRUISE、ALT_HOLD_RTL
- TECS参数：TECS_CLMB_MAX、TECS_SINK_MIN、TECS_SINK_MAX、TECS_TIME_CONST、TECS_THR_DAMP、TECS_PTCH_DAMP、TECS_INTEG_GAIN、TECS_SPDWEIGHT、TECS_RLL2THR、TECS_PITCH_MAX、TECS_PITCH_MIN、TECS_THR_CRUISE、TECS_HDEM_TCONST

**章节来源**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L26-L60)
- [control_params.yaml](file://config/control_params.yaml#L1-L45)

### 参数导入/导出方法
- 导入：ArduPilotParams.from_yaml(path)、AircraftFactory.from_yaml(path)
- 导出：ArduPilotParams.to_yaml(path)、AircraftFactory.export_ardupilot_params(name, output_path, control_yaml)
- 示例：examples/example_6_ardupilot_parameters.py 展示了参数加载、验证、导出与热重载流程

**章节来源**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L74-L99)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L77-L136)
- [example_6_ardupilot_parameters.py](file://examples/example_6_ardupilot_parameters.py#L20-L84)

### 兼容性测试与验证流程
- 参数验证：调用ArduPilotParams.validate()检查参数范围
- 热重载验证：修改ap_params后调用控制器reload_gains()，观察控制响应变化
- 性能对比：示例脚本通过不同PTCH_P增益比较跟踪性能
- 模式切换：在AUTO/LOITER/RTH等模式间切换，验证TECS与L1响应

**章节来源**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L104-L130)
- [example_6_ardupilot_parameters.py](file://examples/example_6_ardupilot_parameters.py#L47-L84)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L152-L168)