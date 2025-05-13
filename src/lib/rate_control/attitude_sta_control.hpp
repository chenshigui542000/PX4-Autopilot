#pragma once
#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/attitude_sta_status.h>
enum
{
	ROLL = 0,
	PITCH,
	YAW
};

class AttitudeStaControl
{
public:
	AttitudeStaControl() = default;
	~AttitudeStaControl() = default;

	matrix::Vector3f update(matrix::Vector3f& attitude_error, const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
		const matrix::Vector3f &angular_accel, const float dt, const bool landed);

	void resetIntW() {_att_sta_w.zero();}

	void resetIntW(size_t axis)
	{
		if(axis < 3){
			_att_sta_w(axis) = 0.f;
		}
	}

	void setStaParams(matrix::Vector3f att_sta_c, matrix::Vector3f att_sta_alpha, matrix::Vector3f att_sta_lamada);

	void setStaWLimit(matrix::Vector3f att_w_limit){
		_att_w_limit = att_w_limit;
	}


	void setStaInertia(matrix::Vector3f att_inertia){_att_inertia = att_inertia;}

	void setStaNormLimit(matrix::Vector3f att_sta_norm_max, matrix::Vector3f att_sta_norm_min);

	void setSaturationStatus(const matrix::Vector3<bool> &saturation_positive,
				 const matrix::Vector3<bool> &saturation_negative);

	void setPositiveSaturationFlag(size_t axis, bool is_saturated);
	void setNegativeSaturationFlag(size_t axis, bool is_saturated);

	void getAttStaStatus(attitude_sta_status_s &att_sta_status);

	matrix::Vector3f getStaTorque(){return _att_torque;}

	matrix::Vector3f getStaW(){return _att_sta_w;}


private:
	void updateIntW(const float dt);

	//moment of inertia
	matrix::Vector3f _att_inertia;
	//params
	matrix::Vector3f _att_sta_c;
	matrix::Vector3f _att_sta_alpha;
	matrix::Vector3f _att_sta_lamada;

	//sliding variable
	matrix::Vector3f _att_ita;

	//torque
	matrix::Vector3f _att_torque;

	//int W
	matrix::Vector3f _att_sta_w;
	matrix::Vector3f _att_sta_w_dot;

	//int limit
	matrix::Vector3f _att_w_limit;

	//norm limit
	matrix::Vector3f _att_sta_norm_max;
	matrix::Vector3f _att_sta_norm_min;


	matrix::Vector<bool, 3> _att_sta_control_saturation_negative;
	matrix::Vector<bool, 3> _att_sta_control_saturation_positive;
};
