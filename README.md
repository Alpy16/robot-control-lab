# Robot Control Lab

A Rust learning project for robot kinematics, joint-space control, fixed-timestep simulation, and safety-oriented state handling.

The project models a two-link planar robotic arm. It accepts a Cartesian target, resolves that target into valid joint angles through analytical inverse kinematics, selects a suitable joint configuration, and moves the simulated arm using a bounded PD controller and a simple physics simulation loop.

This is a focused fundamentals project, not a production robotics stack.

## Current Status

The end-to-end control pipeline is working.

```text
Cartesian target
    ↓
reachability validation
    ↓
analytical inverse kinematics
    ↓
joint-limit filtering
    ↓
nearest valid joint configuration
    ↓
PD acceleration command
    ↓
fixed-timestep simulation
    ↓
updated joint angles and velocities
    ↓
settling check
```

For a two-link arm with unit-length links, the system can move from the initial pose to a target such as `(1.0, 1.0)` and settle near the expected configuration:

```text
shoulder ≈ 0 rad
elbow    ≈ π / 2 rad
```

## Why This Project Exists

The project was built to bridge Rust systems programming with foundational robotics concepts.

A two-link planar arm was chosen because it is small enough to understand fully while still requiring meaningful robotics reasoning:

- forward kinematics
- analytical inverse kinematics
- multiple pose solutions
- reachability checks
- hard and user-defined joint limits
- joint-space control
- fixed-timestep simulation
- feedback and settling conditions
- typed error handling
- state ownership and transactional updates

## Features

- Two-link planar arm geometry
- Forward kinematics
- Analytical inverse kinematics
- Reachability validation
- Elbow-up and elbow-down candidate generation
- Approximate duplicate removal
- Hard physical joint limits
- User-configurable operating limits
- Nearest-solution selection in joint space
- Independent shoulder and elbow PD gains
- Angular-acceleration saturation
- Controlled braking command
- Semi-implicit Euler simulation
- Transactional state updates
- Bounded motion loop with timeout
- Typed geometry, control, and coordinator errors

## Architecture

```text
src/
├── main.rs         # Example construction and execution
├── robot.rs        # Arm geometry, limits, FK, IK, current joint state
├── planner.rs      # Cartesian target to selected joint-space target
├── control.rs      # PD controller, velocities, accelerations, braking
├── simulation.rs   # Simulated plant and fixed-timestep integration
├── coordinator.rs  # Closed-loop orchestration and completion checks
└── errors.rs       # Error types for each subsystem
```

### `robot.rs`

Owns the geometric model and valid joint-position state.

Responsibilities:

- link lengths
- current joint angles
- hard and requested joint limits
- reachability
- forward kinematics
- inverse kinematics
- joint-limit validation
- IK candidate selection

### `planner.rs`

Converts a Cartesian target into one selected joint configuration.

```text
Point2
    ↓
inverse_kinematics
    ↓
Vec<JointAngles>
    ↓
select_nearest_solution
    ↓
JointAngles
```

The current selection cost is:

```text
shoulder_delta² + elbow_delta²
```

This selects the candidate requiring the least joint-space displacement from the current pose. It does not yet optimize for time, energy, obstacle clearance, or distance from joint limits.

### `control.rs`

Contains joint velocities, joint accelerations, and the PD controller.

For each joint:

```text
error = target_angle - current_angle
```

```text
acceleration =
    proportional_gain * error
    - derivative_gain * current_velocity
```

The result is clamped to the configured acceleration limit.

The controller only calculates a command. It does not mutate the arm.

### `simulation.rs`

Represents the simplified physical plant.

It owns:

```text
SimulatedArm
├── TwoLinkArm
└── JointVelocities
```

For each fixed timestep:

```text
next_velocity =
    current_velocity + acceleration * dt
```

```text
next_angle =
    current_angle + next_velocity * dt
```

The simulator calculates a proposed state, validates the proposed angles through the arm's existing limit checks, and only then commits the new velocities.

### `coordinator.rs`

Connects planning, control, and simulation.

For a fixed Cartesian target:

```text
plan once
    ↓
loop:
    read current state
    check settling
    compute acceleration
    advance simulation
    repeat
```

The movement succeeds only when both joints have:

- sufficiently small angle error
- sufficiently small angular velocity

A maximum step count prevents an unstable or poorly tuned controller from looping indefinitely.

## Kinematics

### Forward Kinematics

For link lengths `L1` and `L2`, shoulder angle `θ1`, and relative elbow angle `θ2`:

```text
elbow_x = L1 * cos(θ1)
elbow_y = L1 * sin(θ1)
```

```text
end_x = elbow_x + L2 * cos(θ1 + θ2)
end_y = elbow_y + L2 * sin(θ1 + θ2)
```

### Reachability

For a target at distance `r` from the base:

```text
|L1 - L2| ≤ r ≤ L1 + L2
```

Targets outside this annular region are geometrically unreachable.

A target can still be rejected after passing this check if every IK candidate violates the configured joint limits.

### Inverse Kinematics

The base, elbow, and target form a triangle.

The target direction is:

```text
atan2(y, x)
```

The triangle angles are recovered through the law of cosines. Two mirrored joint configurations may exist:

- elbow-up
- elbow-down

Cosine inputs are clamped to `[-1, 1]` before `acos` to protect against small floating-point errors after reachability has already been validated.

## Control Model

The controller operates independently in joint space.

For each joint:

```text
error = target - current
```

```text
commanded_acceleration =
    Kp * error
    - Kd * angular_velocity
```

Interpretation:

- the proportional term pulls the joint toward the target
- the derivative term opposes motion and provides braking
- acceleration saturation represents a simplified actuator limit

The controller currently commands angular acceleration directly. A real robot would more commonly require a torque or motor-current command and a richer dynamic model.

## Simulation Model

The simulator uses semi-implicit Euler integration:

```text
ω(k+1) = ω(k) + a(k) * dt
```

```text
θ(k+1) = θ(k) + ω(k+1) * dt
```

At `100 Hz`:

```text
dt = 0.01 seconds
```

The acceleration command is recalculated on every tick using the latest angle and velocity state.

## Controlled Stop

The controller also provides a braking command:

```text
stopping_acceleration = -Kd * current_velocity
```

This is a controlled deceleration, not an instantaneous stop. It must be applied repeatedly through the simulator until both joint velocities fall below a stopping tolerance.

A full motion-state machine is not yet implemented.

## Quick Start

### Requirements

- Rust stable
- Cargo

### Run

From the repository root:

```bash
cargo run
```

### Check and format

```bash
cargo check
cargo fmt
cargo clippy
```

### Run tests

```bash
cargo test
```

## Example Configuration

A typical example uses:

```text
link 1 length: 1.0
link 2 length: 1.0

initial shoulder angle: 0.0 rad
initial elbow angle:    0.0 rad

target:
x = 1.0
y = 1.0

timestep:
dt = 0.01 s

maximum steps:
2000
```

The controller gains and tolerances can be adjusted to trade off convergence speed, overshoot, and settling time.

## Design Decisions

### Hard limits and operating limits

The project separates:

- hard physical limits
- user-requested operating limits

Requested limits must remain inside the hard limits. This prevents callers from widening the permitted range beyond the underlying mechanism's safety boundary.

### Joint-space control

The Cartesian target is resolved into joint angles once. The controller then operates directly on shoulder and elbow errors.

This keeps the control layer independent of link geometry and avoids recalculating IK on every tick for a stationary target.

### Transactional simulation updates

The simulator does not commit velocity before validating the proposed joint position.

```text
calculate proposed velocities
    ↓
calculate proposed angles
    ↓
validate angles
    ↓
commit angles
    ↓
commit velocities
```

If angle validation fails, the previous state remains intact.

### Typed subsystem errors

Failures are separated by responsibility:

```text
GeometryError → geometry, limits, reachability, IK
ControlError  → gains, controller inputs, invalid state
MotionError   → orchestration and timeout
```

A dedicated `SimulationError` is a planned cleanup so invalid timesteps and integrated-state failures are no longer represented as geometry errors.

## Current Limitations

- Two-dimensional, two-joint arm only
- No trajectory generation
- No Cartesian path control
- No joint-velocity limits yet
- No obstacle or self-collision detection
- No gravity
- No friction
- No inertia matrix
- No dynamic coupling between joints
- No actuator or motor model
- No sensor noise or latency
- No encoder model
- No watchdog
- No complete emergency-stop state machine
- Limited automated test coverage
- Controller commands acceleration directly rather than torque

The current model is intended to demonstrate the software and control pipeline, not provide physically accurate rigid-body simulation.

## Planned Improvements

1. Expand unit and integration test coverage
2. Add explicit joint-velocity limits
3. Add `Idle`, `Moving`, `Stopping`, and `Faulted` states
4. Integrate controlled stopping into the coordinator
5. Add watchdog and stale-command handling
6. Add telemetry and motion traces
7. Add trajectory generation
8. Add joint-limit-margin penalties to IK selection
9. Add obstacle and collision checks
10. Replace direct acceleration control with torque-level dynamics

A more realistic arm model would eventually include:

```text
torque =
    inertia * acceleration
    + velocity-dependent coupling
    + gravity
```

## Learning Outcomes

This project provided practical experience with:

- translating mechanical equations into Rust
- separating geometry, planning, control, simulation, and orchestration
- modeling state explicitly
- borrowing versus owning shared components
- using `Result` and subsystem-specific errors
- handling floating-point tolerances
- preventing invalid partial state updates
- distinguishing a control command from the physical plant that applies it

## License

MIT
