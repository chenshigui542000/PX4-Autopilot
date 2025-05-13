#include "PositionStaControl.hpp"

/**
 * 位置稳定控制主函数，用于计算竖直方向上的推力命令
 */
float PositionStaControl::update(const float dt, matrix::Vector3f& _pos, matrix::Vector3f& _pos_sp,
	matrix::Vector3f& _vel, matrix::Vector3f& _vel_sp, matrix::Vector3f& _acc_sp)
{
	// 计算中间变量 _pos_ita：结合了位置和速度误差
	_pos_ita = _vel_sp(2) - _vel(2) + _pos_sta_c * (_pos_sp(2) - _pos(2));

	// 检查数值是否有效（非NaN、非Inf），否则清零
	if(!PX4_ISFINITE(_pos_ita)) {
		_pos_ita = 0.f;
	}

	// 获取 _pos_ita 的范数（绝对值）并限制其大小
	float pos_sta_ita_norm;
	pos_sta_ita_norm = math::constrain(fabs(_pos_ita), _pos_sta_norm_min, _pos_sta_norm_max);

	// 计算 _pos_ita 的符号
	float pos_sta_ita_sign;
	pos_sta_ita_sign = _pos_ita / pos_sta_ita_norm;

	// 计算推力命令（竖直方向）
	_pos_thrust = -_mc_mass * _acc_sp(2)                                    // 期望加速度反馈（负值用于 Z 轴朝上）
				+ _mc_mass * _pos_sta_c * (_vel(2) - _vel_sp(2))            // 速度误差
				+ _pos_sta_alpha * sqrt(pos_sta_ita_norm) * pos_sta_ita_sign // STA非线性项
				+ _pos_sta_w;                                                // 积分 w

	// 计算扰动补偿项的微分（w_dot）
	_pos_sta_w_dot = _pos_sta_lamada * pos_sta_ita_sign;

	return _pos_thrust;
}

/**
 * 更新扰动补偿项 w 的积分值，并进行限制
 */
void PositionStaControl::updateIntW(float thr_z_sp, float lim_thr_min, float lim_thr_max, const float dt)
{
	// 检查微分项有效性
	if(!PX4_ISFINITE(_pos_sta_w_dot)) _pos_sta_w_dot = 0.f;

	// 如果已经达到推力限制边界，且积分方向会导致饱和加剧，则停止积分
	if((thr_z_sp >= -lim_thr_min && _pos_ita >= 0)
		||(thr_z_sp <= -lim_thr_max && _pos_ita <= 0))
	{
		_pos_sta_w_dot = 0.f;
	}

	// 积分更新
	_pos_sta_w += _pos_sta_w_dot * dt;

	// 检查积分项有效性
	if(!PX4_ISFINITE(_pos_sta_w)) _pos_sta_w = 0.f;

	// 限制积分项的大小，避免积分饱和
	_pos_sta_w = math::constrain(_pos_sta_w, -_pos_sta_w_limit, _pos_sta_w_limit);
}

/**
 * 计算归一化的速度误差（限幅后除以模长），用于 STA 控制器的 x、y轴
 */
float PositionStaControl::getVelErrorDivNorm(float vel_error, size_t axis)
{
	// 仅处理 x 或 y 轴
	if(axis >= 2) return 0.f;

	// 获取速度误差的绝对值，并进行限幅
	float vel_error_norm = fabs(vel_error);
	vel_error_norm = math::constrain(vel_error_norm, _pos_xy_sta_norm_min(axis), _pos_xy_sta_norm_max(axis));

	// 返回归一化后的速度误差
	return vel_error / vel_error_norm;
}
