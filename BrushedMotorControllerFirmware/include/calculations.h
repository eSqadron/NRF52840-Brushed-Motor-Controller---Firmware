/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Maciej Baczmanski, Jakub Mazur
 */


struct PID_def {
	const uint32_t Kp_numerator;
	const uint32_t Kp_denominator;

	const uint32_t Ki_numerator;
	const uint32_t Ki_denominator;

	const uint32_t Kd_numerator;
	const uint32_t Kd_denominator;

	int32_t i_temp; // error accumulator
	int32_t d_temp; // prev error

	int32_t i_min;
	int32_t i_max;
};

struct PID_def PID_spd_for_pos = {
	.Kp_numerator = CONFIG_KP_NUMERATOR_FOR_SPEED_IN_POS,
	.Kp_denominator = CONFIG_KP_DENOMINATOR_FOR_SPEED_IN_POS,

	.Ki_numerator = 0,
	.Ki_denominator = 1,

	.Kd_numerator = 0,
	.Kd_denominator = 1,

	.i_temp = 0,
	.d_temp = 0,

	.i_min = -1000,
	.i_max = 5000,
};

struct PID_def PID_spd = {
	.Kp_numerator = CONFIG_KP_NUMERATOR_FOR_SPEED,
	.Kp_denominator = CONFIG_KP_DENOMINATOR_FOR_SPEED,

	.Ki_numerator = 0,
	.Ki_denominator = 1,

	.Kd_numerator = 0,
	.Kd_denominator = 1,

	.i_temp = 0,
	.d_temp = 0,

	.i_min = -1000,
	.i_max = 5000,
};

struct PID_def PID_pos = {
	.Kp_numerator = CONFIG_KP_NUMERATOR_FOR_POS,
	.Kp_denominator = CONFIG_KP_DENOMINATOR_FOR_POS,

	.Ki_numerator = 0,
	.Ki_denominator = 1,

	.Kd_numerator = 0,
	.Kd_denominator = 1,

	.i_temp = 0,
	.d_temp = 0,

	.i_min = -500,
	.i_max = 500,
};

#define ERR_C(target_speed, actual_speed) (int32_t)(target_speed - actual_speed)

static inline int32_t calculate_pid_from_error(int32_t error, struct PID_def *pid)
{
	int32_t P = ((int32_t)(((int32_t)pid->Kp_numerator * error) /
				(int32_t)pid->Kp_denominator));
	pid->i_temp = pid->i_temp + error;
	if(pid->i_temp < pid->i_min) pid->i_temp = pid->i_min;
	if(pid->i_temp < pid->i_max) pid->i_temp = pid->i_max;
	int32_t I = ((int32_t)((int32_t)pid->Ki_numerator * pid->i_temp /
			       (int32_t)pid->Ki_denominator));
	int32_t D = ((int32_t)((int32_t)pid->Kd_numerator * (int32_t)(pid->d_temp - error) /
			       (int32_t)pid->Kd_denominator));
	pid->d_temp = error;
	return P + I + D;
}

static inline int32_t calculate_pid(int32_t target_speed, int32_t actual_speed, struct PID_def *pid)
{
	return calculate_pid_from_error(ERR_C(target_speed, actual_speed), pid);
}
