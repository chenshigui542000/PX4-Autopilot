# ISTA Velocity Loop Design (PX4 Port)

Goal: Replace velocity PID law with ISTA per-axis, outputting _acc_sp (m/s^2).

## Mapping (paper -> PX4)
- x (paper x1,k) := vel_error_axis (m/s)
- h := dt (s)
- u (control output) := acc_sp_axis (m/s^2)
- Internal state nu := nu_axis (m/s^2)  (per-axis persistent state)
- lambda1, lambda2: new tunable gains (per-axis or split XY/Z)

## Control placement
Replace ONLY the "velocity control law" inside PositionControl::_velocityControl(dt):
- Current: acc_sp = vel_error*P + _vel_int - _vel_dot*D
- New: acc_sp(axis) = ISTA_update(x=vel_error(axis), h=dt, nu_state(axis), lambda1_axis, lambda2_axis)

Keep:
- dt clamp upstream
- output structure: _acc_sp (Vector3f)
- downstream _accelerationControl()
- all safety constraints / tilt / thrust limits

## Handling D-term (_vel_dot)
ISTA per paper uses x and nu. PX4 currently uses _vel_dot for damping.
Decision for port v1:
- Option A (recommended first): set ISTA output as baseline + keep existing -_vel_dot*D term:
  acc_sp = ISTA(x) - _vel_dot * D
  (D gains kept; ISTA replaces P+I part)
- Option B: remove D entirely (riskier; may change behavior a lot)

Use Option A for the first integration to reduce regression risk.

## Internal state lifecycle (nu reset)
nu_axis must be reset when:
- vehicle disarms
- position controller resets / estimator resets
- mode changes that reset integrators (same place as _vel_int reset)

Implementation: add PositionControl::resetVelocityIsta() called from existing reset paths.

## Saturation / clamping
- Do NOT clamp nu internally unless behind a parameter switch.
- Final _acc_sp must still be constrained by existing acceleration/thrust/tilt logic downstream.
