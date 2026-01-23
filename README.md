

## SuperTwisting2.0 Control Changes

### Affected Control Loops

- Attitude rate loop (mc_rate_control / RateControl): adds AttitudeStaControl, and the STA torque output overrides the PID torque output; the attitude controller publishes `attitude_error` for STA.
- Position/velocity loop (mc_pos_control / PositionControl): adds PositionStaControl and applies STA-style normalization on XY velocity error; Z-axis STA terms are logged but do not directly replace thrust output.

### Modified/Added Files

| File | Change |
| --- | --- |
| [msg/AttitudeAllStatus.msg](msg/AttitudeAllStatus.msg) | New - attitude and setpoint status |
| [msg/AttitudeError.msg](msg/AttitudeError.msg) | New - attitude error |
| [msg/AttitudeStaStatus.msg](msg/AttitudeStaStatus.msg) | New - attitude STA status |
| [msg/PositionStaStatus.msg](msg/PositionStaStatus.msg) | New - position STA status |
| [msg/StaStatus.msg](msg/StaStatus.msg) | New - STA debug status |
| [msg/CMakeLists.txt](msg/CMakeLists.txt) | Modified - add STA-related messages |
| [src/lib/rate_control/attitude_sta_control.hpp](src/lib/rate_control/attitude_sta_control.hpp) | New - attitude STA controller interface |
| [src/lib/rate_control/attitude_sta_control.cpp](src/lib/rate_control/attitude_sta_control.cpp) | New - attitude STA controller implementation |
| [src/lib/rate_control/rate_control.hpp](src/lib/rate_control/rate_control.hpp) | Modified - integrate STA controller |
| [src/lib/rate_control/rate_control.cpp](src/lib/rate_control/rate_control.cpp) | Modified - STA torque output |
| [src/lib/rate_control/CMakeLists.txt](src/lib/rate_control/CMakeLists.txt) | Modified - add STA sources |
| [src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp](src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp) | Modified - expose attitude_error |
| [src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp](src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp) | Modified - compute/cache attitude_error |
| [src/modules/mc_att_control/mc_att_control_main.cpp](src/modules/mc_att_control/mc_att_control_main.cpp) | Modified - publish attitude_error/attitude_all_status |
| [src/modules/mc_rate_control/MulticopterRateControl.hpp](src/modules/mc_rate_control/MulticopterRateControl.hpp) | Modified - STA parameter bindings |
| [src/modules/mc_rate_control/MulticopterRateControl.cpp](src/modules/mc_rate_control/MulticopterRateControl.cpp) | Modified - STA parameter update and status publish |
| [src/modules/mc_rate_control/mc_rate_control_params.c](src/modules/mc_rate_control/mc_rate_control_params.c) | Modified - add ATT_STA_* parameters |
| [src/modules/mc_pos_control/PositionControl/PositionStaControl.hpp](src/modules/mc_pos_control/PositionControl/PositionStaControl.hpp) | New - position STA controller interface |
| [src/modules/mc_pos_control/PositionControl/PositionStaControl.cpp](src/modules/mc_pos_control/PositionControl/PositionStaControl.cpp) | New - position STA controller implementation |
| [src/modules/mc_pos_control/PositionControl/SuperTwisting.hpp](src/modules/mc_pos_control/PositionControl/SuperTwisting.hpp) | New - SuperTwisting debug controller |
| [src/modules/mc_pos_control/PositionControl/SuperTwisting.cpp](src/modules/mc_pos_control/PositionControl/SuperTwisting.cpp) | New - SuperTwisting implementation |
| [src/modules/mc_pos_control/PositionControl/PositionControl.hpp](src/modules/mc_pos_control/PositionControl/PositionControl.hpp) | Modified - integrate PositionStaControl |
| [src/modules/mc_pos_control/PositionControl/PositionControl.cpp](src/modules/mc_pos_control/PositionControl/PositionControl.cpp) | Modified - XY velocity error normalization |
| [src/modules/mc_pos_control/PositionControl/CMakeLists.txt](src/modules/mc_pos_control/PositionControl/CMakeLists.txt) | Modified - add STA sources |
| [src/modules/mc_pos_control/MulticopterPositionControl.hpp](src/modules/mc_pos_control/MulticopterPositionControl.hpp) | Modified - STA parameter bindings |
| [src/modules/mc_pos_control/MulticopterPositionControl.cpp](src/modules/mc_pos_control/MulticopterPositionControl.cpp) | Modified - STA parameter update and status publish |
| [src/modules/mc_pos_control/multicopter_position_control_gain_params.c](src/modules/mc_pos_control/multicopter_position_control_gain_params.c) | Modified - add POS_STA_* parameters |
| [src/modules/logger/logged_topics.cpp](src/modules/logger/logged_topics.cpp) | Modified - add STA log topics |

### Parameters and Defaults

**Attitude rate STA parameters (mc_rate_control)**

| Parameter | Default | Description |
| --- | --- | --- |
| `ATT_STA_R_C` | 0.0 | Roll sliding surface gain c |
| `ATT_STA_P_C` | 0.0 | Pitch sliding surface gain c |
| `ATT_STA_Y_C` | 0.0 | Yaw sliding surface gain c |
| `ATT_STA_R_AL` | 0.15 | Roll nonlinear gain alpha |
| `ATT_STA_P_AL` | 0.15 | Pitch nonlinear gain alpha |
| `ATT_STA_Y_AL` | 0.20 | Yaw nonlinear gain alpha |
| `ATT_STA_R_LA` | 0.20 | Roll disturbance gain lambda |
| `ATT_STA_P_LA` | 0.20 | Pitch disturbance gain lambda |
| `ATT_STA_Y_LA` | 0.10 | Yaw disturbance gain lambda |
| `ATT_STA_R_LIM` | 0.30 | Roll w integrator limit |
| `ATT_STA_P_LIM` | 0.30 | Pitch w integrator limit |
| `ATT_STA_Y_LIM` | 0.30 | Yaw w integrator limit |
| `ATT_STA_R_I` | 0.0030 | Roll inertia (I) |
| `ATT_STA_P_I` | 0.0030 | Pitch inertia (I) |
| `ATT_STA_Y_I` | 0.0 | Yaw inertia (I) |
| `ATT_STA_R_NMAX` | 1.0 | Roll ita normalization max |
| `ATT_STA_P_NMAX` | 1.0 | Pitch ita normalization max |
| `ATT_STA_Y_NMAX` | 1.0 | Yaw ita normalization max |
| `ATT_STA_R_NMIN` | 0.9 | Roll ita normalization min |
| `ATT_STA_P_NMIN` | 0.9 | Pitch ita normalization min |
| `ATT_STA_Y_NMIN` | 0.9 | Yaw ita normalization min |

**Position/velocity STA parameters (mc_pos_control)**

| Parameter | Default | Description |
| --- | --- | --- |
| `POS_STA_C` | 1.0 | Z-axis sliding surface gain c |
| `POS_STA_MASS` | 0.0 | Mass parameter used in Z STA thrust term |
| `POS_STA_AL` | 4.0 | Z-axis nonlinear gain alpha |
| `POS_STA_LA` | 2.0 | Z-axis disturbance gain lambda |
| `POS_STA_W_LIM` | 9.8 | Z-axis w integrator limit |
| `POS_STA_NMAX` | 1.0 | Z-axis ita normalization max |
| `POS_STA_NMIN` | 0.9 | Z-axis ita normalization min |
| `POS_X_STA_NMAX` | 1.0 | X velocity error normalization max |
| `POS_X_STA_NMIN` | 0.9 | X velocity error normalization min |
| `POS_Y_STA_NMAX` | 1.0 | Y velocity error normalization max |
| `POS_Y_STA_NMIN` | 0.9 | Y velocity error normalization min |
