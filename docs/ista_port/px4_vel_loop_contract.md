# PX4 Multicopter Velocity Loop I/O Contract (ISTA Port)

Target: Replace velocity PID in mc_pos_control with ISTA while keeping PX4 interfaces stable.

## Location
- Module: src/modules/mc_pos_control
- Core class: PositionControl
- Function: PositionControl::_velocityControl(const float dt)
- File: src/modules/mc_pos_control/PositionControl/PositionControl.cpp

## Inputs (must remain unchanged)
- Velocity setpoint: _vel_sp
  - Source: trajectory_setpoint_s.velocity[3] (uORB: trajectory_setpoint)
  - Units: m/s
- Velocity measurement: _vel
  - Source: PositionControlStates.velocity, fed from vehicle_local_position_s.vx/vy/vz (filtered)
  - Units: m/s
- Velocity derivative: _vel_dot
  - Source: PositionControlStates.acceleration (velocity derivative for D-term)
  - Units: m/s^2
- dt (ISTA step size h):
  - From MulticopterPositionControl::Run()
  - dt = constrain((timestamp_sample - last_loop) * 1e-6, 0.002, 0.04)
  - Units: s

## Error definition (must remain unchanged)
- vel_error = _vel_sp - _vel  (Vector3f)
- Units: m/s

## Outputs (must remain unchanged)
- Acceleration setpoint: _acc_sp (Vector3f)
  - Units: m/s^2
  - Used downstream by _accelerationControl() to compute _thr_sp and attitude setpoint
- Thrust setpoint: _thr_sp (Vector3f) (computed downstream)
  - Units: normalized thrust vector (PX4 convention)

## Constraints (must be preserved)
- Thrust limits: MPC_THR_MIN, MPC_THR_MAX, MPC_THR_HOVER, MPC_THR_XY_MARG
- Velocity limits: MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN
- Tilt limits: MPC_TILTMAX_AIR, MPC_TILTMAX_LND
- Anti-windup logic currently tied to thrust saturation & vertical limits
  - Any replacement must not remove safety clamping
