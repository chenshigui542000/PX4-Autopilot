#include "attitude_sta_control.hpp"

/**
 * 设置 STA 控制器的核心参数
 * @param att_sta_c 控制增益
 * @param att_sta_alpha 非线性项系数
 * @param att_sta_lamada 积分扰动项系数
 */
void AttitudeStaControl::setStaParams(matrix::Vector3f att_sta_c, matrix::Vector3f att_sta_alpha, matrix::Vector3f att_sta_lamada)
{
	_att_sta_c = att_sta_c;
	_att_sta_alpha = att_sta_alpha;
	_att_sta_lamada = att_sta_lamada;
}

/**
 * 设置 ita 范数的限制（最小值与最大值）
 */
void AttitudeStaControl::setStaNormLimit(matrix::Vector3f att_sta_norm_max, matrix::Vector3f att_sta_norm_min)
{
	_att_sta_norm_max = att_sta_norm_max;
	_att_sta_norm_min = att_sta_norm_min;
}

/**
 * 设置姿态控制输出的饱和状态（正负方向）
 */
void AttitudeStaControl::setSaturationStatus(const matrix::Vector3<bool> &saturation_positive,
	const matrix::Vector3<bool> &saturation_negative)
{
	_att_sta_control_saturation_negative = saturation_negative;
	_att_sta_control_saturation_positive = saturation_positive;
}

/**
 * 设置单轴的正方向饱和标志
 */
void AttitudeStaControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if(axis < 3){
		_att_sta_control_saturation_positive(axis) = is_saturated;
	}
}

/**
 * 设置单轴的负方向饱和标志
 */
void AttitudeStaControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if(axis < 3){
		_att_sta_control_saturation_negative(axis) = is_saturated;
	}
}

/**
 * 获取当前 STA 状态信息，写入到 att_sta_status_s 结构体中
 */
void AttitudeStaControl::getAttStaStatus(attitude_sta_status_s &att_sta_status)
{
	att_sta_status.att_roll_ita = _att_ita(ROLL);
	att_sta_status.att_pitch_ita = _att_ita(PITCH);
	att_sta_status.att_yaw_ita = _att_ita(YAW);

	att_sta_status.att_roll_torque = _att_torque(ROLL);
	att_sta_status.att_pitch_torque = _att_torque(PITCH);
	att_sta_status.att_yaw_torque = _att_torque(YAW);

	att_sta_status.att_roll_w = _att_sta_w(ROLL);
	att_sta_status.att_pitch_w = _att_sta_w(PITCH);
	att_sta_status.att_yaw_w = _att_sta_w(YAW);

	att_sta_status.att_roll_w_dot = _att_sta_w_dot(ROLL);
	att_sta_status.att_pitch_w_dot = _att_sta_w_dot(PITCH);
	att_sta_status.att_yaw_w_dot = _att_sta_w_dot(YAW);
}

/**
 * STA 姿态控制主函数
 * @param attitude_error 姿态误差（期望姿态 - 当前姿态）
 * @param rate 当前角速度
 * @param rate_sp 期望角速度
 * @param angular_accel 当前角加速度（可用于补偿）
 * @param dt 控制周期
 * @param landed 是否处于落地状态（true 表示不执行积分）
 * @return 输出控制力矩（三轴）
 */
matrix::Vector3f AttitudeStaControl::update(matrix::Vector3f& attitude_error, const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
	const matrix::Vector3f &angular_accel, const float dt, const bool landed)
{
	// 避免 NAN 或无效输入
	for(int i = 0;i < 3; ++i){
		if(!PX4_ISFINITE(attitude_error(i))){
			attitude_error(i) = 0.f;
		}
	}

	// 计算中间变量 ita（融合姿态和角速度误差）
	for(int i = 0;i < 3;++i){
		_att_ita(i) = rate_sp(i) - rate(i) + _att_sta_c(i) * attitude_error(i);
	}

	matrix::Vector3f att_ita_norm;

	// 计算 ita 范数并限幅（避免发散）
	for(int i = 0;i < 3;++i){
		att_ita_norm(i) = math::constrain(fabsf(_att_ita(i)), _att_sta_norm_min(i), _att_sta_norm_max(i));
	}

	matrix::Vector3f ita_sign;

	// 获取 ita 的符号，用于非线性函数项
	for(int i = 0;i < 3;++i){
		ita_sign(i) = _att_ita(i) / att_ita_norm(i);
	}

	// 计算输出控制力矩（三轴）
	for(int i = 0;i < 3;++i){
			_att_torque(i) =  _att_inertia(i) * _att_sta_c(i) * (rate(i) - rate_sp(i))  // 阻尼项（角速度误差）
					+ _att_sta_alpha(i) * sqrtf(att_ita_norm(i)) * ita_sign(i)          // 非线性稳定项
			  	+ _att_sta_w(i)                                                    // 扰动补偿积分项
				- _att_inertia(i) * angular_accel(i);                              // 动力学补偿项（角加速度）
	}

	// 更新积分扰动项的导数（w_dot）
	for(int i = 0;i < 3;++i){
		_att_sta_w_dot(i) = _att_sta_lamada(i) * ita_sign(i);
	}

	// 飞行中时执行积分更新，落地时跳过
	if(!landed){
		updateIntW(dt);
	}

	return _att_torque;
}

/**
 * 积分扰动项的更新函数，根据饱和状态判断是否继续积分
 */
void AttitudeStaControl::updateIntW(const float dt)
{
	for(int i = 0;i < 3;++i){

		// 如果当前轴正方向饱和，阻止正向积分
		if(_att_sta_control_saturation_positive(i)){
			_att_sta_w_dot(i) = math::min(_att_sta_w_dot(i), 0.f);
		}

		// 如果当前轴负方向饱和，阻止负向积分（注意逻辑）
		if(_att_sta_control_saturation_negative(i)){
			_att_sta_w_dot(i) = math::max(_att_sta_w_dot(i), 1.f); // **疑似应为 0.f，此处建议检查**
		}

		// 计算积分因子：误差越大时抑制积分（防止积分发散）
		float i_factor = _att_ita(i) / math::radians(400.f); // 将误差归一化
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor); // 误差越大，i_factor 越小

		// 积分更新
		float sta_w = _att_sta_w(i) +  i_factor * _att_sta_w_dot(i) * dt;

		// 检查有效性并限幅
		if(PX4_ISFINITE(sta_w)){
			_att_sta_w(i) = math::constrain(sta_w, -_att_w_limit(i), _att_w_limit(i));
		}
	}
}
