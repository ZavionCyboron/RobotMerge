# RobotMerge

Primary FRC robot codebase for **Early College High School / Hangar 84**.  
Written in **Kotlin** using **WPILib’s command-based framework**, RobotMerge supports
multiple drivetrain types (MAXSwerve and Mecanum), autonomous path following, rich
telemetry, and a custom-built simulation environment.

---

## Overview

RobotMerge is the team’s main robot software project, designed to unify drivetrain
experimentation, autonomous development, telemetry, and simulation into a single,
maintainable codebase.

A hardware boot selector or simulation chooser determines whether the robot runs as
a **Swerve** or **Mecanum** drive system, allowing shared commands, controls, and
autonomous logic across both implementations.

---

## Key Features

- Kotlin + WPILib **command-based architecture**
- Runtime selection between **MAXSwerve** and **Mecanum** drivetrains
- Autonomous routines using **PathPlanner** and **AutoBuilder**
- Real-time telemetry via **SmartDashboard / NetworkTables**
- Custom simulation framework with:
  - ground-truth vs estimated pose comparison
  - sensor noise and bias modeling
  - configurable start poses
- Field visualization using `Field2d`
- Modular drivetrain, command, and telemetry abstractions

---

## Drivetrain Selection

### Real Robot
- A **digital input (DIO 19)** selects the drivetrain at boot:
  - LOW → `SWERVE`
  - HIGH → `MECANUM`

### Simulation
- A **SmartDashboard chooser** (`Sim/RobotType`) selects the drivetrain type at runtime.

This allows testing both drivebases without changing code.

---

## Controls (Xbox Controller – Port 0)

### Driving
- **Swerve Mode**
  - Left Stick Y → forward/back
  - Left Stick X → strafe
  - Right Stick X → rotation
  - Left Bumper (hold) → park / lock wheels
- **Mecanum Mode**
  - Left Stick X → strafe
  - Left Stick Y → forward/back
  - Right Stick X → rotation

Inputs use:
- deadband (`0.08`)
- squared response curves
- slew-rate limiting for smooth control

### Launcher
- **Left Trigger (hold)** → intake
- **Right Trigger (hold)** → launch
- Release → stop

---

## Autonomous

Autonomous routines are built using **PathPlanner** and executed via
WPILib’s **AutoBuilder**:

- Path-based autonomous commands
- Alliance-aware path flipping
- Odometry-based starting pose alignment
- Unified autonomous interface for both drivetrains

An autonomous chooser is published to SmartDashboard and populated by the active
drivetrain implementation.

---

## Telemetry & Monitoring

### Runtime Telemetry
Published continuously via **NetworkTables / SmartDashboard**:

- Robot pose:
  - `X`, `Y`, `HeadingDeg`
- Gyro:
  - `YawDeg`, `YawRateDegPerSec`
- Chassis speeds:
  - `Vx`, `Vy`, `Omega`
- Wheel/module encoder positions and velocities (FL / FR / RL / RR)
- Driver Station state:
  - Enabled / Auto / Teleop
  - Match time

Telemetry is designed for live debugging, tuning, and validation.

---

## Simulation Framework

RobotMerge includes a **custom simulation system**, not just default WPILib sim.

### Simulation Capabilities

- Deterministic simulation clock (`SimClock`)
- Ground-truth robot pose tracking
- Estimated pose tracking (odometry)
- Realistic sensor modeling:
  - gyro noise
  - gyro rate noise
  - bias drift over time
- Truth vs measured comparison telemetry
- Configurable drivetrain selection in sim
- Field visualization via `Field2d`

### Pose Comparison Telemetry

The following simulation-only telemetry is published:

- True pose vs estimated pose
- Pose error:
  - ΔX
  - ΔY
  - ΔYaw
  - XY distance error
  - Angular error
- True vs measured gyro yaw and yaw rate

This allows direct validation of odometry, estimation accuracy, and autonomous
behavior before deploying to hardware.

---

## Simulation Start Poses

Predefined field start poses are available for both alliances:

- Left / Middle / Right (Red)
- Left / Middle / Right (Blue)

Each pose:
- resets odometry
- zeros the gyro
- sets ground-truth and estimated pose
- updates the Field2d display

---

## Repository Structure
```text
src/
├── main/
│   ├──deploy/
│   |   ├── pathplanner/
│   |   |    ├── autos/           # Autonomous routines
│   |   |    ├── paths/           # PathPlanner path files
│   |   |    ├── navgrid.json     # Navigation grid
│   |   |
|   |   └── telemetry/
|   |            └── telemetry.json # Telemetry configuration
│   └── kotlin/
│       └── org/hangar84/robot2026/
│           ├── commands/        # Reusable drive and robot commands
│           ├── constants/       # Robot constants and configuration values
│           ├── io/              # Robot hardware interfaces
│           │    ├── real/       # Actual robot hardware
│           │    └── sim/        # Simulated robot hardware
│           ├── mecanum/         # Mecanum drivetrain implementation
│           ├── sim/             # Custom simulation framework (state, sensors, field)
│           ├── subsystems/      # Shared subsystem interfaces and implementations
│           ├── swerve/          # MAXSwerve drivetrain implementation
│           ├── telemetry/       # Telemetry and simulation telemetry helpers
│           ├── Main.kt          # Robot entry point
│           ├── Robot.kt         # Robot lifecycle logic
│           └── RobotContainer.kt# Input bindings, drivetrain selection, autos
```

## Getting Started

1. Install WPILib for the current FRC season
2. Clone this repository
3. Open in WPILib VS Code
4. Build and deploy, or run **Simulate Robot Code**

---

## Contributing (Team Workflow)

- Branch naming:
  - `feature/...`
  - `fix/...`
- Validate behavior in simulation before merging
- Keep telemetry keys stable to avoid dashboard breakage

---

## Credits

Early College High School Robotics / Hangar 84  
Built with **WPILib**, **PathPlanner**, and vendor libraries.
