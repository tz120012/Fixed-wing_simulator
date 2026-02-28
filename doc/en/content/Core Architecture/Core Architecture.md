# Core Architecture

<cite>
**Referenced Files in This Document**
- [main.py](file://main.py)
- [simulator.py](file://src/simulation/simulator.py)
- [state_manager.py](file://src/simulation/state_manager.py)
- [aircraft_factory.py](file://src/models/aircraft_factory.py)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py)
- [navigation_controller.py](file://src/control/navigation_controller.py)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py)
- [wind_model.py](file://src/environment/wind_model.py)
- [animator.py](file://src/visualization/animator.py)
- [config_loader.py](file://src/utils/config_loader.py)
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py)
- [integrator.py](file://src/simulation/integrator.py)
</cite>

## Table of Contents
1. [Introduction](#introduction)
2. [Project Structure](#project-structure)
3. [Core Components](#core-components)
4. [Architecture Overview](#architecture-overview)
5. [Detailed Component Analysis](#detailed-component-analysis)
6. [Dependency Analysis](#dependency-analysis)
7. [Performance Considerations](#performance-considerations)
8. [Troubleshooting Guide](#troubleshooting-guide)
9. [Conclusion](#conclusion)
10. [Appendices](#appendices)

## Introduction
This document describes the core architecture of the FixedWingSimulator system. It explains the high-level design patterns (layered architecture, factory pattern, and observer-like control flow), the modular structure separating simulation engine, aircraft models, control systems, environment modeling, planning, and visualization, and the orchestration mechanisms that coordinate runtime behavior. It also documents system boundaries, data flows, responsibilities, technical decisions, architectural trade-offs, and design principles.

## Project Structure
The project is organized into feature-based packages under src/, each encapsulating a domain area:
- simulation: orchestration, state management, numerical integration
- models: aircraft configuration via factory and database
- control: flight modes, navigation, attitude/rate control, servo mixing, TECS
- dynamics: nonlinear 6-DOF equations, linearized models, aerodynamics
- environment: wind and atmospheric models
- planning: waypoints and trajectory generation
- visualization: plotting and animation
- utils: configuration loader and shared utilities

Entry point is main.py, which constructs the FixedWingSimulator and executes simulations or analyses.

```mermaid
graph TB
A_main["main.py<br/>CLI entry point"] --> B_sim["FixedWingSimulator<br/>src/simulation/simulator.py"]
B_sim --> C_models["Models<br/>aircraft_factory.py"]
B_sim --> D_dynamics["Dynamics<br/>nonlinear_model.py"]
B_sim --> E_env["Environment<br/>wind_model.py"]
B_sim --> F_control["Control<br/>flight_mode_manager.py<br/>navigation_controller.py"]
B_sim --> G_plan["Planning<br/>waypoint_manager.py"]
B_sim --> H_sim["Simulation internals<br/>state_manager.py<br/>integrator.py"]
B_sim --> Viz["Visualization<br/>animator.py"]
B_sim --> U["Utils<br/>config_loader.py"]
```

**Diagram sources**
- [main.py](file://main.py#L98-L144)
- [simulator.py](file://src/simulation/simulator.py#L115-L642)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L39-L136)
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L104-L386)
- [wind_model.py](file://src/environment/wind_model.py#L18-L113)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L20-L208)
- [state_manager.py](file://src/simulation/state_manager.py#L16-L193)
- [integrator.py](file://src/simulation/integrator.py#L17-L108)
- [animator.py](file://src/visualization/animator.py#L14-L150)
- [config_loader.py](file://src/utils/config_loader.py#L59-L82)

**Section sources**
- [main.py](file://main.py#L1-L145)
- [simulator.py](file://src/simulation/simulator.py#L1-L642)

## Core Components
- FixedWingSimulator orchestrates the entire simulation lifecycle, wiring aircraft, environment, planning, control, dynamics, and visualization.
- AircraftFactory builds AircraftConfig from database and optional overrides.
- FlightModeManager selects and transitions between flight modes, producing ControlTarget commands.
- NavigationController computes lateral roll command via L1 and vertical/energy commands via TECS.
- WaypointManager manages NED waypoints and builds trajectories.
- NonlinearModel defines 6-DOF equations of motion and computes trim.
- Wind model provides time-varying NED wind vectors.
- StateHistory records time histories efficiently; AircraftSimState holds derived quantities.
- Integrator provides real-time step-wise ODE integration.
- ConfigLoader merges YAML configurations with defaults.
- Visualization modules provide quick plots and animations.

**Section sources**
- [simulator.py](file://src/simulation/simulator.py#L115-L642)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L39-L136)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L20-L208)
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L104-L386)
- [wind_model.py](file://src/environment/wind_model.py#L18-L113)
- [state_manager.py](file://src/simulation/state_manager.py#L16-L193)
- [integrator.py](file://src/simulation/integrator.py#L17-L108)
- [config_loader.py](file://src/utils/config_loader.py#L59-L82)
- [animator.py](file://src/visualization/animator.py#L14-L150)

## Architecture Overview
The system follows a layered architecture:
- Presentation/CLI layer (main.py)
- Orchestration layer (FixedWingSimulator)
- Domain layers (models, dynamics, control, planning, environment)
- Infrastructure (visualization, config loader, integrator)

Design patterns:
- Factory pattern: AircraftFactory constructs AircraftConfig from database and overrides.
- Observer-like control flow: FlightModeManager produces ControlTarget; NavigationController consumes it; Attitude/Rate controllers consume ControlTarget; ServoMixer converts to normalized actuator outputs.
- Strategy pattern: WaypointManager delegates trajectory construction to concrete trajectory classes.

```mermaid
graph TB
subgraph "Presentation"
CLI["main.py"]
end
subgraph "Orchestration"
SIM["FixedWingSimulator"]
end
subgraph "Domain"
MODELS["AircraftFactory"]
DYN["NonlinearModel"]
CTRL["FlightModeManager<br/>NavigationController"]
PLAN["WaypointManager"]
ENV["Wind"]
end
subgraph "Infrastructure"
STATE["StateHistory<br/>AircraftSimState"]
INT["Integrator"]
CFG["ConfigLoader"]
VIZ["Animator"]
end
CLI --> SIM
SIM --> MODELS
SIM --> DYN
SIM --> CTRL
SIM --> PLAN
SIM --> ENV
SIM --> STATE
SIM --> INT
SIM --> CFG
SIM --> VIZ
```

**Diagram sources**
- [main.py](file://main.py#L98-L144)
- [simulator.py](file://src/simulation/simulator.py#L115-L642)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L39-L136)
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L104-L386)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L20-L208)
- [wind_model.py](file://src/environment/wind_model.py#L18-L113)
- [state_manager.py](file://src/simulation/state_manager.py#L16-L193)
- [integrator.py](file://src/simulation/integrator.py#L17-L108)
- [config_loader.py](file://src/utils/config_loader.py#L59-L82)
- [animator.py](file://src/visualization/animator.py#L14-L150)

## Detailed Component Analysis

### FixedWingSimulator Orchestration
FixedWingSimulator is the central coordinator:
- Loads configuration via ConfigLoader
- Builds aircraft via AircraftFactory
- Instantiates environment (Wind), control layers (FlightModeManager, NavigationController, Attitude/Rate controllers, ServoMixer), planning (WaypointManager), and dynamics (NonlinearModel)
- Runs closed-loop simulation with real-time stepping using Dopri5Integrator
- Produces SimulationResult with StateHistory and trim metadata

Key runtime flow:
- Computes trim, initializes integrator, resets control layers
- For each step: reads state, computes ControlTarget via FlightModeManager, updates NavigationController, then Attitude/Rate/ServoMixer, records history, integrates ODE

```mermaid
sequenceDiagram
participant CLI as "main.py"
participant SIM as "FixedWingSimulator"
participant FM as "FlightModeManager"
participant NAV as "NavigationController"
participant AC as "AttitudeController"
participant RC as "RateController"
participant SM as "ServoMixer"
participant DYN as "NonlinearModel"
participant WIND as "Wind"
participant INT as "Dopri5Integrator"
participant HIST as "StateHistory"
CLI->>SIM : construct with config, aircraft, wind, traj
SIM->>SIM : compute trim, init integrator
loop for each time step
SIM->>INT : step(dt)
INT-->>SIM : y(t+dt)
SIM->>DYN : state_dot(t,y,controls,wind,rho)
DYN-->>SIM : ydot
SIM->>FM : update(state, nav_target)
FM-->>SIM : ControlTarget
SIM->>NAV : update(state, segment)
NAV-->>SIM : ControlTarget
SIM->>AC : update(phi,theta,psi,roll_cmd,pitch_cmd,...)
AC-->>SIM : rate_cmd
SIM->>RC : update(p,q,r,roll_rate_cmd,pitch_rate_cmd,...)
RC-->>SIM : servo_cmd
SIM->>SM : update(rate_outputs,elevator,...)
SM-->>SIM : ServoOutput
SIM->>HIST : record(t,state,servo_output,des_pos)
end
SIM-->>CLI : SimulationResult
```

**Diagram sources**
- [simulator.py](file://src/simulation/simulator.py#L239-L567)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L173-L211)
- [navigation_controller.py](file://src/control/navigation_controller.py#L134-L200)
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L160-L200)
- [wind_model.py](file://src/environment/wind_model.py#L76-L108)
- [integrator.py](file://src/simulation/integrator.py#L50-L71)
- [state_manager.py](file://src/simulation/state_manager.py#L124-L174)

**Section sources**
- [simulator.py](file://src/simulation/simulator.py#L115-L642)

### Aircraft Model Factory Pattern
AircraftFactory merges database parameters with optional YAML and dict overrides, producing AircraftConfig consumed by the simulation engine. It also supports exporting ArduPilot-compatible parameter sets.

```mermaid
classDiagram
class AircraftFactory {
+create(name, yaml_overrides, param_overrides) AircraftConfig
+from_yaml(config_path) AircraftConfig
+export_ardupilot_params(name, output_path, control_yaml) void
}
class AircraftConfig {
+string name
+dict aero_params
+summary() string
}
AircraftFactory --> AircraftConfig : "produces"
```

**Diagram sources**
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L39-L136)

**Section sources**
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L39-L136)

### Control Layer: Flight Modes and Navigation
FlightModeManager selects and transitions between modes (MANUAL, STABILIZE, FBW_A, FBW_B, AUTO, LOITER, RTH), producing ControlTarget. NavigationController computes lateral roll via L1 and vertical/energy commands via TECS, returning ControlTarget consumed by attitude/rate controllers.

```mermaid
classDiagram
class FlightModeManager {
+FlightMode current_mode
+FlightMode previous_mode
+set_mode(new_mode) void
+set_mode_str(mode_str) void
+update(state, nav_target, dt) ControlTarget
}
class ControlTarget {
+float roll_cmd
+float pitch_cmd
+float yaw_cmd
+float roll_rate_cmd
+float pitch_rate_cmd
+float airspeed_cmd
+float altitude_cmd
+float throttle_cmd
+bool is_direct
}
class NavigationController {
+update(state, segment, dt) ControlTarget
+reset(state) void
}
class PathSegment {
+ndarray start
+ndarray end
+float target_speed
}
FlightModeManager --> ControlTarget : "produces"
NavigationController --> ControlTarget : "produces"
NavigationController --> PathSegment : "consumes"
```

**Diagram sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L200)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L25-L43)

**Section sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L200)

### Planning and Trajectory Management
WaypointManager stores NED waypoints, loads/saves from YAML, and builds AbstractTrajectory instances (MinimumSnap/MimimumJerk). It exposes desired_state(t) and active segment accessors for real-time tracking.

```mermaid
flowchart TD
Start(["Build trajectory"]) --> CheckWPs["Check ≥2 waypoints"]
CheckWPs --> |<2| Error["Raise ValueError"]
CheckWPs --> |≥2| LoopCheck["Loop enabled?"]
LoopCheck --> |Yes| CloseLoop["Append first WP to close"]
LoopCheck --> |No| KeepWPs["Keep waypoints"]
CloseLoop --> BuildType{"Traj type?"}
KeepWPs --> BuildType
BuildType --> |minimum_snap| Snap["Instantiate MinimumSnapTrajectory"]
BuildType --> |minimum_jerk| Jerk["Instantiate MinimumJerkTrajectory"]
Snap --> Cache["Cache trajectory"]
Jerk --> Cache
Cache --> Done(["Return AbstractTrajectory"])
```

**Diagram sources**
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L127-L167)

**Section sources**
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L20-L208)

### Dynamics and Environment
NonlinearModel defines 6-DOF equations of motion, computes trim, and evaluates state derivatives given Controls, wind in body frame, and air density. Wind model provides NED wind vectors according to configured type (NONE, FIXED, SINE, RANDOMSINE).

```mermaid
classDiagram
class NonlinearModel {
+compute_trim() TrimResult
+state_dot(t, y, controls, wind_body, rho) ndarray
}
class Controls {
+float elevator
+float aileron
+float rudder
+float throttle
}
class TrimResult {
+float alpha_trim
+float de_trim
+float U0
}
class Wind {
+get_wind_ned(t) ndarray
}
NonlinearModel --> Controls : "consumes"
NonlinearModel --> TrimResult : "produces"
Wind --> NonlinearModel : "supplies wind_body"
```

**Diagram sources**
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L104-L200)
- [wind_model.py](file://src/environment/wind_model.py#L76-L108)

**Section sources**
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L104-L200)
- [wind_model.py](file://src/environment/wind_model.py#L18-L113)

### State Management and Integration
StateHistory pre-allocates arrays for efficient recording; AircraftSimState encapsulates 12-D state plus derived quantities (alpha, beta, airspeed, altitude). Integrator provides a real-time step-wise ODE solver (Dopri5) suitable for closed-loop simulation.

```mermaid
classDiagram
class AircraftSimState {
+float u,v,w
+float p,q,r
+float phi,theta,psi
+float x_north,x_east,x_down
+float alpha,beta,airspeed,altitude
+from_array(arr) AircraftSimState
+to_array() ndarray
}
class StateHistory {
+record(t, state, elevator, aileron, rudder, throttle, des_pos) void
+trim() void
+to_dict() dict
}
class Dopri5Integrator {
+step(dt) ndarray
+t float
+y ndarray
}
StateHistory --> AircraftSimState : "records"
Dopri5Integrator --> NonlinearModel : "evaluates state_dot"
```

**Diagram sources**
- [state_manager.py](file://src/simulation/state_manager.py#L16-L193)
- [integrator.py](file://src/simulation/integrator.py#L17-L71)
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L160-L200)

**Section sources**
- [state_manager.py](file://src/simulation/state_manager.py#L16-L193)
- [integrator.py](file://src/simulation/integrator.py#L17-L108)

### Visualization
Visualization modules provide quick 2D and 3D views. The SimulationResult wrapper can trigger plotting and animation using the plotter and animator.

```mermaid
sequenceDiagram
participant SIM as "SimulationResult"
participant PLOT as "FixedWingPlotter"
participant ANIM as "FixedWingAnimator"
SIM->>PLOT : plot_6dof_matplotlib(history, name, show)
SIM->>ANIM : animate(history, name, show)
```

**Diagram sources**
- [simulator.py](file://src/simulation/simulator.py#L92-L109)
- [animator.py](file://src/visualization/animator.py#L25-L150)

**Section sources**
- [simulator.py](file://src/simulation/simulator.py#L59-L110)
- [animator.py](file://src/visualization/animator.py#L14-L150)

## Dependency Analysis
The FixedWingSimulator composes subsystems with clear boundaries:
- models depends on database and YAML
- dynamics depends on aerodynamics and math utilities
- control depends on flight mode manager and TECS
- planning depends on trajectory implementations
- simulation depends on integrator and state containers
- visualization depends on history data
- environment supplies wind to dynamics

```mermaid
graph LR
CLI["main.py"] --> SIM["simulator.py"]
SIM --> MODELS["aircraft_factory.py"]
SIM --> DYN["nonlinear_model.py"]
SIM --> CTRL["flight_mode_manager.py<br/>navigation_controller.py"]
SIM --> PLAN["waypoint_manager.py"]
SIM --> ENV["wind_model.py"]
SIM --> SIMINF["state_manager.py<br/>integrator.py"]
SIM --> VIZ["animator.py"]
SIM --> UTIL["config_loader.py"]
```

**Diagram sources**
- [main.py](file://main.py#L98-L144)
- [simulator.py](file://src/simulation/simulator.py#L33-L51)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L12-L12)
- [nonlinear_model.py](file://src/dynamics/nonlinear_model.py#L27-L28)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L41-L43)
- [navigation_controller.py](file://src/control/navigation_controller.py#L20-L21)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L15-L17)
- [wind_model.py](file://src/environment/wind_model.py#L14-L15)
- [state_manager.py](file://src/simulation/state_manager.py#L9-L13)
- [integrator.py](file://src/simulation/integrator.py#L14-L14)
- [animator.py](file://src/visualization/animator.py#L10-L11)
- [config_loader.py](file://src/utils/config_loader.py#L7-L7)

**Section sources**
- [simulator.py](file://src/simulation/simulator.py#L33-L51)

## Performance Considerations
- Real-time closed-loop simulation uses an adaptive step ODE solver with a fixed-step interface, balancing accuracy and latency.
- StateHistory pre-allocates arrays to minimize memory churn during recording.
- Wind and atmospheric computations are lightweight; density is computed per step based on altitude.
- Trajectory caching avoids recomputation when waypoints are unchanged.
- Visualization is optional and offloaded to external libraries.

[No sources needed since this section provides general guidance]

## Troubleshooting Guide
Common issues and remedies:
- Integration failure: The integrator raises a runtime error when integration fails; check initial conditions, control saturation, and trim computation.
- Unknown aircraft: AircraftFactory validates against known names; ensure the aircraft exists in the database.
- Trajectory errors: WaypointManager requires at least two waypoints; otherwise a ValueError is raised.
- Wind type validation: Wind constructor validates supported types; incorrect values raise an error.
- Control parameter validation: ArduPilot parameters are validated; missing or invalid entries may require YAML configuration.

**Section sources**
- [simulator.py](file://src/simulation/simulator.py#L558-L562)
- [aircraft_factory.py](file://src/models/aircraft_factory.py#L140-L141)
- [waypoint_manager.py](file://src/planning/waypoint_manager.py#L139-L140)
- [wind_model.py](file://src/environment/wind_model.py#L40-L41)
- [integrator.py](file://src/simulation/integrator.py#L54-L56)

## Conclusion
The FixedWingSimulator employs a clean layered architecture with strong separation of concerns. The FixedWingSimulator orchestrates aircraft, environment, planning, control, dynamics, and visualization modules. Design patterns like factory and observer-like control flow enable modularity and extensibility. The system balances real-time performance with configurability and provides robust mechanisms for state management, integration, and visualization.

## Appendices

### System Context Diagram
```mermaid
graph TB
subgraph "External"
User["User"]
end
subgraph "System"
CLI["main.py"]
SIM["FixedWingSimulator"]
subgraph "Subsystems"
M["Models"]
D["Dynamics"]
C["Control"]
P["Planning"]
E["Environment"]
S["Simulation internals"]
V["Visualization"]
U["Utils"]
end
end
User --> CLI
CLI --> SIM
SIM --> M
SIM --> D
SIM --> C
SIM --> P
SIM --> E
SIM --> S
SIM --> V
SIM --> U
```

**Diagram sources**
- [main.py](file://main.py#L98-L144)
- [simulator.py](file://src/simulation/simulator.py#L115-L642)