# Codex Tasklist: Replace PX4 Velocity PID with ISTA (mc_pos_control)

## Preconditions
- Read docs/ista_port/px4_vel_loop_contract.md
- Read docs/ista_port/ista_vel_loop_design.md
- Read docs/ista/bbsta_ista_key.md (paper key equations)
- (Optional) Read existing ISTA implementation notes if present

## Step 0: Locate and snapshot current behavior (no logic change)
- Open PositionControl::_velocityControl(dt)
- Identify the exact block computing vel_error, _acc_sp, _vel_int update, and anti-windup
- Add temporary comments marking: "BEGIN velocity control law" and "END velocity control law"
Acceptance:
- No functional change, builds clean.

## Step 1: Add ISTA helper class (standalone, unit-testable)
- Add new files under src/modules/mc_pos_control/ (or a shared lib folder):
  - ISTA.hpp / ISTA.cpp (or rename to IstaController.* to avoid collisions)
- API:
  - float update(float x, float h);    // returns u
  - setters/getters for lambda1/lambda2, reset(), get_nu()
- Ensure it compiles in PX4 build system (CMakeLists / module build file)
Acceptance:
- Builds clean, no usage yet.

## Step 2: Add parameters (lambda1/lambda2) + enable switch
- Add new PX4 params:
  - MPC_VEL_ISTA_EN (bool/int): 0=PID, 1=ISTA
  - MPC_VEL_ISTA_L1_XY, MPC_VEL_ISTA_L2_XY
  - MPC_VEL_ISTA_L1_Z,  MPC_VEL_ISTA_L2_Z
  - (Optional) MPC_VEL_ISTA_KEEP_D (bool) default 1
- Wire param update into MulticopterPositionControl and pass to PositionControl
Acceptance:
- Params visible, default keeps existing behavior (ISTA disabled).

## Step 3: Integrate ISTA into _velocityControl(dt) behind switch
When MPC_VEL_ISTA_EN == 1:
- Compute vel_error = _vel_sp - _vel (unchanged)
- For each axis:
  - u_ista = ista_axis.update(x=vel_error(axis), h=dt)
- Build _acc_sp:
  - If KEEP_D: _acc_sp = u_ista_vector - _vel_dot.emult(_gain_vel_d)
  - Else:      _acc_sp = u_ista_vector
- Keep existing anti-windup & constraints blocks as much as possible:
  - Vertical integrator clamp: disable _vel_int update when ISTA is enabled OR repurpose _vel_int to 0
  - Horizontal ARW: keep thrust saturation-based feedback but apply to nu carefully OR disable initially with a note
Acceptance:
- Builds clean
- With ISTA disabled: identical outputs (bitwise close) to baseline
- With ISTA enabled: _acc_sp produced and downstream logic untouched

## Step 4: Reset behavior
- Add reset path for ISTA nu states:
  - When existing code resets velocity integrator, also reset ISTA
  - On disarm: reset ISTA
Acceptance:
- No stale nu after disarm/rearm in logs.

## Step 5: Add minimal logging and SITL validation hooks
- Add optional debug (behind compile flag or parameter):
  - log: vel_error, acc_sp, nu, case_id per axis
- Provide a SITL test plan in docs:
  - Hover + step velocity setpoint (x/y/z)
  - Wind disturbance
Acceptance:
- Able to confirm ISTA engaged and stable in SITL.

## Step 6: Safety checks
- Ensure no NaNs:
  - guard dt>0, finite checks
- Ensure output constraints still apply
Acceptance:
- No NaNs in flight log; thrust/tilt limits respected.
