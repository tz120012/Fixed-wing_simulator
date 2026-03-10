# Flight Mode Management

<cite>
**Referenced Files in This Document**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py)
- [navigation_controller.py](file://src/control/navigation_controller.py)
- [tecs_controller.py](file://src/control/tecs_controller.py)
- [simulator.py](file://src/simulation/simulator.py)
- [state_manager.py](file://src/simulation/state_manager.py)
- [math_utils.py](file://src/utils/math_utils.py)
- [control_params.yaml](file://config/control_params.yaml)
- [main.py](file://main.py)
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
This document describes the five-layer ArduPilot-compatible flight mode management system implemented in the FixedWingSimulator. It covers the FlightModeManager class, the ControlTarget data structure, and the AircraftState dataclass used for mode decision-making. It explains how modes are selected and executed, how navigation controllers integrate with the mode manager, and how parameters are configured to achieve stable and predictable behavior across MANUAL, STABILIZE, FBW_A, FBW_B, AUTO, LOITER, and RTH modes. Practical examples demonstrate mode switching, parameter configuration, and integration with the control layers.

## Project Structure
The flight mode management system resides in the control subsystem and integrates with the simulation engine and navigation controllers. The main entry point demonstrates how modes are selected at startup.

```mermaid
graph TB
subgraph "Control Layer"
FMM["FlightModeManager<br/>src/control/flight_mode_manager.py"]
NC["NavigationController<br/>src/control/navigation_controller.py"]
TECS["TECSController<br/>src/control/tecs_controller.py"]
APC["ArdupilotParams<br/>src/control/ardupilot_compat.py"]
end
subgraph "Simulation Engine"
SIM["FixedWingSimulator<br/>src/simulation/simulator.py"]
ACS["AircraftState<br/>src/control/flight_mode_manager.py"]
CT["ControlTarget<br/>src/control/flight_mode_manager.py"]
AS["AircraftSimState<br/>src/simulation/state_manager.py"]
end
subgraph "Utilities"
MU["math_utils<br/>src/utils/math_utils.py"]
end
subgraph "Configuration"
CFG["control_params.yaml<br/>config/control_params.yaml"]
CLI["main.py<br/>main.py"]
end
CLI --> SIM
SIM --> FMM
SIM --> NC
SIM --> TECS
SIM --> APC
SIM --> AS
FMM --> ACS
FMM --> CT
NC --> TECS
NC --> MU
APC --> CFG
```

**Diagram sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [tecs_controller.py](file://src/control/tecs_controller.py#L50-L647)
- [simulator.py](file://src/simulation/simulator.py#L115-L642)
- [state_manager.py](file://src/simulation/state_manager.py#L16-L94)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L130)
- [math_utils.py](file://src/utils/math_utils.py#L1-L124)
- [control_params.yaml](file://config/control_params.yaml#L1-L45)
- [main.py](file://main.py#L98-L145)

**Section sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L1-L298)
- [simulator.py](file://src/simulation/simulator.py#L115-L642)
- [main.py](file://main.py#L98-L145)

## Core Components
- FlightModeManager: Orchestrates mode selection and computes ControlTarget for each control step. It supports MANUAL, STABILIZE, FBW_A, FBW_B, AUTO, LOITER, and RTH.
- ControlTarget: A data structure carrying desired angles, rates, speeds, altitudes, and throttle commands. It also supports direct control overrides for MANUAL mode.
- AircraftState: A minimal snapshot of the aircraft’s state (position, velocity, attitude, angular rates, airspeed, altitude) used by the mode manager and navigation controller.
- NavigationController: Implements L1 lateral guidance and TECS altitude/airspeed control, producing ControlTarget for AUTO/LOITER/RTH.
- TECSController: Implements ArduPilot-compatible Total Energy Control System for altitude and airspeed regulation.
- ArdupilotParams: Parameter container mirroring ArduPilot naming conventions, loaded from YAML and validated.

**Section sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L28-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [tecs_controller.py](file://src/control/tecs_controller.py#L50-L647)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L130)

## Architecture Overview
The control loop integrates the mode manager with navigation and control layers. The simulator constructs the mode manager and navigation controller from ArduPilot-compatible parameters, then at each step:
- Converts the full simulation state to AircraftState for the control system.
- Computes a navigation target (ControlTarget) from NavigationController.
- Calls FlightModeManager.update to produce a ControlTarget for the current mode.
- Executes attitude and rate control, then servo mixing to produce normalized servo outputs.
- Converts normalized servo outputs to physical controls and advances the dynamics.

```mermaid
sequenceDiagram
participant CLI as "CLI (main.py)"
participant SIM as "FixedWingSimulator"
participant FMM as "FlightModeManager"
participant NC as "NavigationController"
participant ATT as "AttitudeController"
participant RATE as "RateController"
participant SERVO as "ServoMixer"
participant DYN as "NonlinearModel"
CLI->>SIM : Initialize with initial_mode
SIM->>FMM : Construct with cruise_speed/cruise_alt
SIM->>NC : Construct with NAVL1 params and TECS params
loop Every control step
SIM->>SIM : Build AircraftState from AircraftSimState
SIM->>NC : update(AircraftState, PathSegment)
NC-->>SIM : ControlTarget (nav_target)
SIM->>FMM : update(AircraftState, nav_target, dt)
FMM-->>SIM : ControlTarget (mode_target)
alt mode is DIRECT
SIM->>SERVO : update(mode_target)
SERVO-->>SIM : ServoOutput
else mode requires control
SIM->>ATT : update(state angles, mode_target)
ATT-->>SIM : roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd
SIM->>RATE : update(state rates, att_out)
RATE-->>SIM : elevator, aileron, rudder
SIM->>SERVO : update(rate_out, throttle_cmd)
SERVO-->>SIM : ServoOutput
end
SIM->>DYN : state_dot(..., Controls)
DYN-->>SIM : next state
end
```

**Diagram sources**
- [simulator.py](file://src/simulation/simulator.py#L416-L565)
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L173-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L134-L202)
- [tecs_controller.py](file://src/control/tecs_controller.py#L247-L315)

## Detailed Component Analysis

### FlightModeManager
- Responsibilities:
  - Tracks current and previous mode.
  - Transitions modes and logs transitions.
  - Computes ControlTarget for each mode.
  - Captures loiter center on entry to LOITER.
  - Exposes manual stick inputs for MANUAL mode.
- Modes:
  - MANUAL: Direct pass-through of stick inputs; sets is_direct=True.
  - STABILIZE: Hold wings-level; uses nav_target pitch/throttle/altitude if provided.
  - FBW_A: Hold current roll, maintain altitude with pitch; simulate stick-to-angle behavior.
  - FBW_B: Altitude hold + airspeed hold; uses nav_target pitch/throttle/altitude if provided.
  - AUTO/LOITER/RTH: Use nav_target if available; fallback to hold current state; LOITER captures center on first update; RTH builds a nav_target with cruise altitude if none provided.
- Transition logic:
  - set_mode updates previous/current mode and resets loiter capture on LOITER entry.
  - update dispatches to per-mode handlers; fallback returns cruise-speed/cruise-altitude targets.

```mermaid
classDiagram
class FlightModeManager {
+FlightMode current_mode
+FlightMode previous_mode
+ndarray home_pos_ned
+float cruise_speed
+float cruise_alt
-ndarray _loiter_pos
+float manual_elevator
+float manual_aileron
+float manual_rudder
+float manual_throttle
+set_mode(new_mode)
+set_mode_str(mode_str)
+update(state, nav_target, dt) ControlTarget
-_manual(state) ControlTarget
-_stabilize(state, nav_target) ControlTarget
-_fbw_a(state) ControlTarget
-_fbw_b(state, nav_target) ControlTarget
-_auto(state, nav_target) ControlTarget
}
class FlightMode {
<<enumeration>>
+MANUAL
+STABILIZE
+FBW_A
+FBW_B
+AUTO
+LOITER
+RTH
}
class ControlTarget {
+float roll_cmd
+float pitch_cmd
+float yaw_cmd
+float roll_rate_cmd
+float pitch_rate_cmd
+float yaw_rate_cmd
+float airspeed_cmd
+float altitude_cmd
+Optional[float] elevator_direct
+Optional[float] aileron_direct
+Optional[float] rudder_direct
+Optional[float] throttle_direct
+float throttle_cmd
+bool is_direct
}
class AircraftState {
+float pos_north
+float pos_east
+float pos_down
+float u
+float v
+float w
+float phi
+float theta
+float psi
+float p
+float q
+float r
+float airspeed
+float altitude
+pos_ned() ndarray
+vel_body() ndarray
+euler() ndarray
+omega() ndarray
}
FlightModeManager --> FlightMode : "uses"
FlightModeManager --> ControlTarget : "produces"
FlightModeManager --> AircraftState : "consumes"
```

**Diagram sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L28-L298)

**Section sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)

### ControlTarget Data Structure
- Purpose: Carries desired commands from the mode manager to the control layers.
- Fields:
  - Desired angles and rates for attitude control.
  - Airspeed and altitude targets.
  - Optional direct control overrides for MANUAL mode.
  - Throttle command and a flag indicating whether to bypass attitude/rate control.

Integration:
- Mode manager produces ControlTarget for each mode.
- Navigation controller writes lateral roll and yaw commands and altitude/airspeed targets.
- Attitude and rate controllers consume ControlTarget to compute actuator commands.

**Section sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L82-L113)

### AircraftState Dataclass
- Purpose: Minimal snapshot of the aircraft’s state for control decisions.
- Provides convenient properties for position, velocity, Euler angles, and angular rates.
- Used by FlightModeManager and NavigationController.

**Section sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L38-L80)
- [state_manager.py](file://src/simulation/state_manager.py#L16-L94)

### NavigationController and TECS
- NavigationController:
  - Implements L1 lateral navigation law and TECS altitude/airspeed control.
  - Produces ControlTarget with roll_cmd, yaw_cmd, pitch_cmd, throttle_cmd, airspeed_cmd, altitude_cmd.
  - Uses PathSegment to define desired path and target speed.
- TECSController:
  - Implements ArduPilot-style Total Energy Control System.
  - Computes throttle and pitch commands from height, climb rate, airspeed, and demands.
  - Includes underspeed detection, bad descent detection, and energy shaping.

```mermaid
flowchart TD
Start(["NavigationController.update"]) --> L1["Compute L1 roll command"]
L1 --> ClampRoll["Clamp roll to max_roll"]
ClampRoll --> Heading["Set yaw_cmd to segment direction"]
Heading --> Alt["Set altitude_cmd from segment end"]
Alt --> Climb["Estimate climb rate from u,w,theta"]
Climb --> Accel["Estimate body-x acceleration"]
Accel --> TECS["TECS.update(height, climb_rate, airspeed, accel_body_x, roll_rad, hgt_dem, airspeed_dem, dt)"]
TECS --> PitchThrottle["Write pitch_cmd and throttle_cmd"]
PitchThrottle --> Output["Return ControlTarget"]
```

**Diagram sources**
- [navigation_controller.py](file://src/control/navigation_controller.py#L134-L202)
- [tecs_controller.py](file://src/control/tecs_controller.py#L247-L315)

**Section sources**
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [tecs_controller.py](file://src/control/tecs_controller.py#L50-L647)

### Mode-Specific Behaviors and Defaults
- MANUAL:
  - Direct stick-to-servo pass-through; is_direct=True.
  - Manual inputs are stored in FlightModeManager and applied in _manual.
- STABILIZE:
  - Hold wings-level (roll_cmd=0); use nav_target pitch/throttle/altitude if provided.
  - Uses cruise_speed and cruise_alt as fallbacks.
- FBW_A:
  - Hold current roll and maintain altitude with pitch; simulate stick-to-angle behavior.
  - Uses cruise_speed and cruise_alt as fallbacks.
- FBW_B:
  - Altitude hold + airspeed hold; use nav_target pitch/throttle/altitude if provided.
  - Uses cruise_speed and cruise_alt as fallbacks.
- AUTO/LOITER/RTH:
  - Use nav_target if provided; fallback to hold current state.
  - LOITER captures center on first update; RTH builds a nav_target with cruise altitude if none provided.

Parameter defaults:
- Cruise speed and altitude are taken from ArduPilotParams (AIRSPEED_CRUISE and ALT_HOLD_RTL).
- Navigation parameters (NAVL1_PERIOD, NAVL1_DAMPING) and TECS parameters are loaded from control_params.yaml.

**Section sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L194-L298)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L17-L130)
- [control_params.yaml](file://config/control_params.yaml#L1-L45)

### Integration with Simulation Engine
- FixedWingSimulator constructs FlightModeManager and NavigationController using ArduPilot-compatible parameters.
- At each step, it converts AircraftSimState to AircraftState, computes nav_target, calls FlightModeManager.update, and executes the control layers.
- Servo outputs are converted to Controls and passed to the dynamics.

**Section sources**
- [simulator.py](file://src/simulation/simulator.py#L173-L233)
- [simulator.py](file://src/simulation/simulator.py#L416-L565)

## Dependency Analysis
- FlightModeManager depends on:
  - AircraftState and ControlTarget for I/O.
  - ArduPilot parameter defaults for cruise speed/altitude.
- NavigationController depends on:
  - TECSController for altitude/airspeed control.
  - math_utils for angle wrapping and saturation.
- Simulator orchestrates:
  - FlightModeManager, NavigationController, AttitudeController, RateController, ServoMixer, and NonlinearModel.
  - Loads ArduPilotParams from YAML and validates them.

```mermaid
graph TB
FMM["FlightModeManager"]
ACS["AircraftState"]
CT["ControlTarget"]
APC["ArdupilotParams"]
NC["NavigationController"]
TECS["TECSController"]
MU["math_utils"]
SIM["FixedWingSimulator"]
SIM --> FMM
SIM --> NC
SIM --> APC
FMM --> ACS
FMM --> CT
NC --> TECS
NC --> MU
```

**Diagram sources**
- [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L115-L298)
- [navigation_controller.py](file://src/control/navigation_controller.py#L45-L293)
- [tecs_controller.py](file://src/control/tecs_controller.py#L50-L647)
- [simulator.py](file://src/simulation/simulator.py#L173-L233)

**Section sources**
- [simulator.py](file://src/simulation/simulator.py#L173-L233)
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L74-L98)

## Performance Considerations
- TECS smoothing and rate limiting:
  - TECS uses time constants and damping gains to reduce oscillations and improve stability.
  - Height demand is low-pass filtered to prevent abrupt jumps.
- L1 guidance:
  - Look-ahead distance scales with airspeed; prevents overshoot and improves tracking.
- Mode transitions:
  - The mode manager does not implement explicit transition smoothing; transitions occur on mode set. For smoother transitions, consider interpolating ControlTarget values across a short window during mode changes.
- Parameter tuning:
  - NAVL1 damping and period influence lateral stability and aggressiveness.
  - TECS time constant and damping influence vertical response smoothness.

[No sources needed since this section provides general guidance]

## Troubleshooting Guide
- Mode not switching:
  - Ensure set_mode or set_mode_str is called with a different mode than current_mode.
  - Verify that the simulator loop calls FlightModeManager.update each step.
- Stalls or oscillations:
  - Check TECS parameters (time constant, damping, pitch limits).
  - Verify NAVL1 damping and period.
  - Confirm airspeed limits and cruise throttle are reasonable for the aircraft.
- Unexpected altitude behavior:
  - Confirm ALT_HOLD_RTL and AIRSPEED_CRUISE in control_params.yaml.
  - Ensure TECS thr_cruise aligns with computed trim.
- Overshoot or undershoot in AUTO/LOITER/RTH:
  - Adjust TECS sink/climb rates and speed weight.
  - Verify PathSegment altitudes and target speeds.

**Section sources**
- [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L104-L129)
- [navigation_controller.py](file://src/control/navigation_controller.py#L120-L131)
- [tecs_controller.py](file://src/control/tecs_controller.py#L197-L246)

## Conclusion
The flight mode management system provides a clean, ArduPilot-compatible framework for fixed-wing control. FlightModeManager encapsulates mode selection and ControlTarget production, while NavigationController and TECS deliver robust lateral and vertical control. Parameters are configurable via YAML and validated for safe operation. Together, these components enable stable AUTO, LOITER, and RTH behaviors, as well as pilot-in-the-loop modes like MANUAL, STABILIZE, and FBW_A/B.

[No sources needed since this section summarizes without analyzing specific files]

## Appendices

### Practical Examples

- Mode switching at runtime:
  - Call FlightModeManager.set_mode or set_mode_str with a new FlightMode to trigger a transition. The manager logs the transition and prepares mode-specific state (e.g., capturing loiter center).
  - Example invocation path: [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L152-L167)

- Parameter configuration:
  - Configure ArduPilot-compatible parameters in control_params.yaml (e.g., NAVL1_PERIOD, NAVL1_DAMPING, TECS_*).
  - Load and validate parameters via ArdupilotParams.from_yaml and validate.
  - Example path: [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L82-L98), [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L104-L129)
  - Example YAML: [control_params.yaml](file://config/control_params.yaml#L1-L45)

- Integration with navigation controllers:
  - Build a PathSegment and call NavigationController.update to produce ControlTarget for AUTO/LOITER/RTH.
  - Example path: [navigation_controller.py](file://src/control/navigation_controller.py#L134-L202)

- Starting with a specific mode:
  - Use the CLI to select initial mode; the simulator constructs FlightModeManager with the chosen mode.
  - Example path: [main.py](file://main.py#L45-L48), [simulator.py](file://src/simulation/simulator.py#L174-L179)

- Mode-specific defaults:
  - Cruise speed and altitude defaults come from ArduPilotParams; fallback targets are produced when no nav_target is provided.
  - Example path: [flight_mode_manager.py](file://src/control/flight_mode_manager.py#L210-L211), [ardupilot_compat.py](file://src/control/ardupilot_compat.py#L58-L59)